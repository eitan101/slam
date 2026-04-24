// --- UTILITIES & MATH ---

// Generate Gaussian distributed random numbers (Box-Muller transform)
function randn_bm() {
    let u = 0, v = 0;
    while(u === 0) u = Math.random(); // Converting [0,1) to (0,1)
    while(v === 0) v = Math.random();
    let num = Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
    return num;
}

// Normalize angle to [-PI, PI]
function normalizeAngle(angle) {
    while (angle > Math.PI) angle -= 2.0 * Math.PI;
    while (angle < -Math.PI) angle += 2.0 * Math.PI;
    return angle;
}

// Distance between two points
function distance(p1, p2) {
    return Math.sqrt(Math.pow(p1.x - p2.x, 2) + Math.pow(p1.y - p2.y, 2));
}

// Gaussian Probability Density Function
function gaussianPdf(x, mu, sigma) {
    const variance = sigma * sigma;
    return (1.0 / Math.sqrt(2.0 * Math.PI * variance)) * Math.exp(-Math.pow(x - mu, 2) / (2.0 * variance));
}


// --- SIMULATION STATE ---

const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');

let config = {
    dt: 0.05, // Simulation time step
    motionNoiseV: 0.1, // Velocity noise multiplier
    motionNoiseW: 0.1, // Angular velocity noise multiplier
    sensorNoiseRange: 5.0, // Range noise in pixels
    sensorNoiseBearing: 0.05, // Bearing noise in radians
    sensorRange: 250, // Max range of LIDAR/Sensors
    sensorFOV: Math.PI * 2, // 360 degree sensor
    numParticles: 150,
    numLandmarks: 20,
    algo: 'fastslam', // 'odometry', 'mcl', 'fastslam'
    autoDrive: true
};

let state = {
    realRobot: { x: 0, y: 0, theta: 0 },
    odomRobot: { x: 0, y: 0, theta: 0 },
    estRobot: { x: 0, y: 0, theta: 0 },
    controls: { v: 0, w: 0 },
    landmarks: [], // {x, y}
    measurements: [], // Active measurements from ground truth
    particles: [], // SLAM/MCL particles
    pathReal: [], // History for drawing
    pathOdom: [],
    pathEst: [],
    lastTime: 0,
    frameCount: 0,
    fpsTime: 0
};

const keys = { w: false, a: false, s: false, d: false, ArrowUp: false, ArrowLeft: false, ArrowDown: false, ArrowRight: false };


class Particle {
    constructor(x, y, theta, weight) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.weight = weight;
        // For FastSLAM: Each particle has its own estimate of landmarks
        // Element: { x, y, hits, id }
        this.map = []; 
    }

    clone() {
        let p = new Particle(this.x, this.y, this.theta, this.weight);
        // Deep copy map for FastSLAM
        p.map = this.map.map(lm => ({...lm}));
        return p;
    }
}

function initSimulation() {
    const container = canvas.parentElement;
    // Use a fallback of 800x600 if the layout hasn't fully computed yet
    canvas.width = Math.max(container.clientWidth, 800);
    canvas.height = Math.max(container.clientHeight, 600);

    const cx = canvas.width / 2;
    const cy = canvas.height / 2;

    // Reset robots to center
    state.realRobot = { x: cx, y: cy, theta: 0 };
    state.odomRobot = { x: cx, y: cy, theta: 0 };
    state.estRobot = { x: cx, y: cy, theta: 0 };
    
    state.pathReal = [];
    state.pathOdom = [];
    state.pathEst = [];
    state.measurements = [];

    state.landmarks = [];
    for(let i=0; i<config.numLandmarks; i++) {
        let lx, ly;
        let attempts = 0; // Safeguard against infinite loop if canvas is too small
        do {
            lx = Math.random() * (canvas.width - 100) + 50;
            ly = Math.random() * (canvas.height - 100) + 50;
            attempts++;
        } while(distance({x: cx, y: cy}, {x: lx, y: ly}) < 100 && attempts < 50);
        
        state.landmarks.push({ x: lx, y: ly, id: i });
    }

    // Initialize particles
    initParticles();
}

function initParticles() {
    state.particles = [];
    const initialWeight = 1.0 / config.numParticles;
    for(let i = 0; i < config.numParticles; i++) {
        // Initialize around starting position
        state.particles.push(new Particle(
            state.odomRobot.x, 
            state.odomRobot.y, 
            state.odomRobot.theta, 
            initialWeight
        ));
    }
}


function updateKinematics() {
    // 1. Handle Input & Controls
    if (config.autoDrive) {
        // Simple wandering algorithm
        let time = performance.now() / 1000;
        state.controls.v = 60; // pixels per sec
        state.controls.w = Math.sin(time * 0.5) * 0.8 + Math.cos(time * 0.2) * 0.3;
        
        // Avoid walls
        let margin = 100;
        let nx = state.realRobot.x + Math.cos(state.realRobot.theta) * 50;
        let ny = state.realRobot.y + Math.sin(state.realRobot.theta) * 50;
        if (nx < margin || nx > canvas.width - margin || ny < margin || ny > canvas.height - margin) {
            state.controls.w = 1.5; // Turn sharply away
        }
    } else {
        // Manual control
        state.controls.v = 0;
        state.controls.w = 0;
        if (keys.w || keys.ArrowUp) state.controls.v = 80;
        if (keys.s || keys.ArrowDown) state.controls.v = -80;
        if (keys.a || keys.ArrowLeft) state.controls.w = -1.5;
        if (keys.d || keys.ArrowRight) state.controls.w = 1.5;
    }

    let { v, w } = state.controls;
    let dt = config.dt;

    // 2. Update Ground Truth (Real Robot)
    state.realRobot.x += v * Math.cos(state.realRobot.theta) * dt;
    state.realRobot.y += v * Math.sin(state.realRobot.theta) * dt;
    state.realRobot.theta = normalizeAngle(state.realRobot.theta + w * dt);

    // Record history for trail
    if (state.frameCount % 5 === 0) {
        state.pathReal.push({...state.realRobot});
        if(state.pathReal.length > 200) state.pathReal.shift();
    }

    // 3. Update Odometry (Dead Reckoning with noise)
    // Odometry sensors aren't perfect, they accumulate error
    let odom_v = v + (v !== 0 ? randn_bm() * config.motionNoiseV * Math.abs(v) : 0);
    let odom_w = w + (w !== 0 ? randn_bm() * config.motionNoiseW * Math.abs(w) : 0);
    
    state.odomRobot.x += odom_v * Math.cos(state.odomRobot.theta) * dt;
    state.odomRobot.y += odom_v * Math.sin(state.odomRobot.theta) * dt;
    state.odomRobot.theta = normalizeAngle(state.odomRobot.theta + odom_w * dt);

    if (state.frameCount % 5 === 0) {
        state.pathOdom.push({...state.odomRobot});
        if(state.pathOdom.length > 200) state.pathOdom.shift();
    }

    // 4. Generate Sensor Measurements (Simulate LIDAR/Cameras)
    state.measurements = [];
    for (let lm of state.landmarks) {
        let dist = distance(state.realRobot, lm);
        if (dist <= config.sensorRange) {
            let bearing = Math.atan2(lm.y - state.realRobot.y, lm.x - state.realRobot.x) - state.realRobot.theta;
            bearing = normalizeAngle(bearing);
            
            // Add measurement noise
            let z_range = dist + randn_bm() * config.sensorNoiseRange;
            let z_bearing = normalizeAngle(bearing + randn_bm() * config.sensorNoiseBearing);
            
            state.measurements.push({ range: z_range, bearing: z_bearing, id: lm.id });
        }
    }
}


function updateSLAM() {
    if (config.algo === 'odometry') {
        // Just use odometry as the estimate
        state.estRobot = {...state.odomRobot};
        return;
    }

    let { v, w } = state.controls;
    let dt = config.dt;
    let weightsSum = 0;

    // PREDICTION STEP: Move particles according to motion model
    for (let p of state.particles) {
        // Add noise to particle motion (particles represent uncertainty)
        let pv = v + randn_bm() * config.motionNoiseV * Math.abs(v) * 2.0; 
        let pw = w + randn_bm() * config.motionNoiseW * Math.abs(w) * 2.0;

        p.x += pv * Math.cos(p.theta) * dt;
        p.y += pv * Math.sin(p.theta) * dt;
        p.theta = normalizeAngle(p.theta + pw * dt);
    }

    // UPDATE STEP: Weigh particles based on measurements
    if (state.measurements.length > 0) {
        for (let p of state.particles) {
            let prob = 1.0;

            for (let z of state.measurements) {
                if (config.algo === 'mcl') {
                    // MCL: Map is known. Find corresponding known landmark.
                    let lm = state.landmarks.find(l => l.id === z.id);
                    if (lm) {
                        let expected_range = distance(p, lm);
                        let expected_bearing = normalizeAngle(Math.atan2(lm.y - p.y, lm.x - p.x) - p.theta);
                        
                        let prob_r = gaussianPdf(z.range, expected_range, config.sensorNoiseRange);
                        let prob_b = gaussianPdf(z.bearing, expected_bearing, config.sensorNoiseBearing);
                        prob *= (prob_r * prob_b);
                    }
                } 
                else if (config.algo === 'fastslam') {
                    // FastSLAM: Map is unknown. Data association & mapping per particle.
                    
                    // Calculate where the observation puts the landmark relative to this particle
                    let obs_lx = p.x + z.range * Math.cos(p.theta + z.bearing);
                    let obs_ly = p.y + z.range * Math.sin(p.theta + z.bearing);

                    // Find closest existing landmark in particle's map
                    let minDist = Infinity;
                    let bestLm = null;
                    
                    for (let lm of p.map) {
                        let d = distance({x: obs_lx, y: obs_ly}, lm);
                        if (d < minDist) {
                            minDist = d;
                            bestLm = lm;
                        }
                    }

                    const DATA_ASSOCIATION_THRESHOLD = 50; // pixels

                    if (bestLm && minDist < DATA_ASSOCIATION_THRESHOLD) {
                        // Known landmark: Calculate weight and update landmark
                        let expected_range = distance(p, bestLm);
                        let expected_bearing = normalizeAngle(Math.atan2(bestLm.y - p.y, bestLm.x - p.x) - p.theta);
                        
                        let prob_r = gaussianPdf(z.range, expected_range, config.sensorNoiseRange);
                        let prob_b = gaussianPdf(z.bearing, expected_bearing, config.sensorNoiseBearing);
                        
                        // Prevent prob from being 0
                        prob *= Math.max(1e-10, prob_r * prob_b);

                        // Simple Running Average update for landmark position (Simplified EKF)
                        bestLm.hits += 1;
                        let learningRate = 1.0 / bestLm.hits;
                        bestLm.x = bestLm.x * (1 - learningRate) + obs_lx * learningRate;
                        bestLm.y = bestLm.y * (1 - learningRate) + obs_ly * learningRate;
                    } else {
                        // New landmark discovery: Add to map
                        p.map.push({ x: obs_lx, y: obs_ly, hits: 1, id: Math.random() });
                        // Default probability penalty for adding a new landmark to prevent over-mapping
                        prob *= 0.001; 
                    }
                }
            }
            
            p.weight *= prob;
            weightsSum += p.weight;
        }

        // NORMALIZE WEIGHTS
        if (weightsSum > 0) {
            for (let p of state.particles) {
                p.weight /= weightsSum;
            }
        } else {
            // If all weights are 0 (lost), reset uniformly
            let uniform = 1.0 / config.numParticles;
            for (let p of state.particles) p.weight = uniform;
        }

        // RESAMPLING STEP (Stochastic Universal Resampling / Systematic Resampling)
        // Resample when effective particle size is small to prevent degeneracy
        let neff = 1.0 / state.particles.reduce((sum, p) => sum + p.weight * p.weight, 0);
        
        if (neff < config.numParticles / 2.0) {
            let newParticles = [];
            let maxWeight = Math.max(...state.particles.map(p => p.weight));
            let beta = 0;
            let index = Math.floor(Math.random() * config.numParticles);
            
            for (let i = 0; i < config.numParticles; i++) {
                beta += Math.random() * 2.0 * maxWeight;
                while (beta > state.particles[index].weight) {
                    beta -= state.particles[index].weight;
                    index = (index + 1) % config.numParticles;
                }
                let clonedP = state.particles[index].clone();
                clonedP.weight = 1.0 / config.numParticles;
                newParticles.push(clonedP);
            }
            state.particles = newParticles;
        }
    }

    // ESTIMATION: Calculate expected pose (Weighted Mean)
    let estX = 0, estY = 0, estThetaX = 0, estThetaY = 0;
    
    // Find best particle for map extraction
    let bestParticle = state.particles[0];

    for (let p of state.particles) {
        estX += p.x * p.weight;
        estY += p.y * p.weight;
        estThetaX += Math.cos(p.theta) * p.weight;
        estThetaY += Math.sin(p.theta) * p.weight;
        if (p.weight > bestParticle.weight) {
            bestParticle = p;
        }
    }
    
    state.estRobot = { 
        x: estX, 
        y: estY, 
        theta: Math.atan2(estThetaY, estThetaX) 
    };

    // Attach best map to estimate for drawing
    if (config.algo === 'fastslam') {
        state.estRobot.map = bestParticle.map;
    } else {
        state.estRobot.map = state.landmarks; // If known map
    }

    if (state.frameCount % 5 === 0) {
        state.pathEst.push({x: estX, y: estY});
        if(state.pathEst.length > 200) state.pathEst.shift();
    }
}


function drawRobot(r, color, isReal=false) {
    ctx.save();
    ctx.translate(r.x, r.y);
    ctx.rotate(r.theta);
    
    ctx.beginPath();
    // Triangle shape
    ctx.moveTo(15, 0);
    ctx.lineTo(-10, 10);
    ctx.lineTo(-10, -10);
    ctx.closePath();
    
    if (isReal) {
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
        // Direction line
        ctx.beginPath();
        ctx.moveTo(0,0);
        ctx.lineTo(25, 0);
        ctx.strokeStyle = color;
        ctx.stroke();
    } else {
        ctx.fillStyle = color;
        ctx.fill();
    }
    ctx.restore();
}

function drawPath(path, color, isDashed=false) {
    if(path.length < 2) return;
    ctx.beginPath();
    ctx.moveTo(path[0].x, path[0].y);
    for(let i=1; i<path.length; i++) {
        ctx.lineTo(path[i].x, path[i].y);
    }
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    if(isDashed) ctx.setLineDash([5, 5]);
    ctx.stroke();
    ctx.setLineDash([]); // Reset
}

function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 1. Draw Paths
    drawPath(state.pathReal, '#3b82f6'); // Blue real path
    drawPath(state.pathOdom, '#ef4444', true); // Red dashed odometry
    if (config.algo !== 'odometry') {
        drawPath(state.pathEst, '#34d399'); // Emerald estimated path
    }

    // 2. Draw Real Landmarks
    ctx.fillStyle = '#94a3b8'; // Slate 400
    for (let lm of state.landmarks) {
        ctx.fillRect(lm.x - 4, lm.y - 4, 8, 8);
    }

    // 3. Draw Sensor Rays
    ctx.strokeStyle = 'rgba(59, 130, 246, 0.2)'; // Faint blue
    ctx.lineWidth = 1;
    for (let z of state.measurements) {
        ctx.beginPath();
        ctx.moveTo(state.realRobot.x, state.realRobot.y);
        // Reconstruct absolute point from ground truth for visualizer
        let lx = state.realRobot.x + z.range * Math.cos(state.realRobot.theta + z.bearing);
        let ly = state.realRobot.y + z.range * Math.sin(state.realRobot.theta + z.bearing);
        ctx.lineTo(lx, ly);
        ctx.stroke();
    }

    // 4. Draw Particles
    if (config.algo !== 'odometry') {
        ctx.fillStyle = 'rgba(250, 204, 21, 0.3)'; // Yellowish
        for (let p of state.particles) {
            ctx.beginPath();
            ctx.arc(p.x, p.y, 2, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    // 5. Draw Estimated Map (FastSLAM)
    if (config.algo === 'fastslam' && state.estRobot.map) {
        ctx.strokeStyle = '#34d399'; // Emerald
        ctx.lineWidth = 2;
        for (let lm of state.estRobot.map) {
            if (lm.hits > 2) { // Only draw confident landmarks
                ctx.beginPath();
                ctx.arc(lm.x, lm.y, 6, 0, Math.PI * 2);
                ctx.stroke();
            }
        }
    }

    // 6. Draw Robots
    drawRobot(state.odomRobot, 'rgba(239, 68, 68, 0.5)'); // Ghost Red Odom
    if (config.algo !== 'odometry') {
        drawRobot(state.estRobot, '#34d399'); // Emerald Estimate
    }
    drawRobot(state.realRobot, '#3b82f6', true); // Real Robot (outline)
}

function gameLoop(timestamp) {
    if (!state.lastTime) state.lastTime = timestamp;
    let elapsed = timestamp - state.lastTime;
    
    // Target roughly fixed timestep physics updates
    if (elapsed > 1000/60) {
        // Update performance counter
        if (state.frameCount % 30 === 0) {
            document.getElementById('fpsCount').innerText = Math.round(1000 / elapsed);
        }

        state.lastTime = timestamp;
        state.frameCount++;

        updateKinematics();
        updateSLAM();
        render();
    }
    
    requestAnimationFrame(gameLoop);
}


// Setup UI interactivity
function setupUI() {
    const algos = {
        'odometry': "Dead Reckoning simply integrates velocity commands to estimate position. Without reference points, sensor noise causes errors to accumulate infinitely over time.",
        'mcl': "Monte Carlo Localization uses a known map. A swarm of particles guesses the robot's pose. Particles that 'see' landmarks where the map says they should be are kept and multiplied.",
        'fastslam': "Simultaneous Localization and Mapping. The robot doesn't know where it is or where the landmarks are. Each particle guesses a pose AND builds its own internal map of the world."
    };

    const updateAlgoDesc = () => {
        config.algo = document.getElementById('algoSelect').value;
        document.getElementById('algoDesc').innerText = algos[config.algo];
        document.getElementById('modeDisplay').innerText = document.getElementById('algoSelect').options[document.getElementById('algoSelect').selectedIndex].text;
        if (config.algo !== 'odometry' && state.particles.length === 0) {
            initParticles();
        }
    };

    document.getElementById('algoSelect').addEventListener('change', updateAlgoDesc);
    updateAlgoDesc();

    document.getElementById('btnReset').addEventListener('click', initSimulation);
    
    const btnAutoDrive = document.getElementById('btnAutoDrive');
    btnAutoDrive.addEventListener('click', () => {
        config.autoDrive = !config.autoDrive;
        if (config.autoDrive) {
            btnAutoDrive.innerText = "Auto-Drive: ON";
            btnAutoDrive.className = "flex-1 bg-blue-600 hover:bg-blue-500 text-white py-2 rounded-md text-sm transition-colors font-medium shadow-lg shadow-blue-500/20";
        } else {
            btnAutoDrive.innerText = "Auto-Drive: OFF";
            btnAutoDrive.className = "flex-1 bg-slate-700 hover:bg-slate-600 text-white py-2 rounded-md text-sm transition-colors";
        }
    });

    // Sliders
    const bindSlider = (idSlider, idVal, configKey, isInt=false) => {
        const el = document.getElementById(idSlider);
        const val = document.getElementById(idVal);
        el.addEventListener('input', (e) => {
            let v = parseFloat(e.target.value);
            if (isInt) v = parseInt(v);
            config[configKey] = v;
            // Exception for motion noise variables splitting
            if (configKey === 'motionNoiseV') config.motionNoiseW = v;
            
            val.innerText = isInt ? v : v.toFixed(2);

            if (configKey === 'numParticles') initParticles();
        });
    };

    bindSlider('slMotionNoise', 'valMotionNoise', 'motionNoiseV');
    bindSlider('slSensorNoise', 'valSensorNoise', 'sensorNoiseRange');
    bindSlider('slParticles', 'valParticles', 'numParticles', true);
    bindSlider('slSensorRange', 'valSensorRange', 'sensorRange', true);

    // Keyboard
    window.addEventListener('keydown', (e) => {
        if (keys.hasOwnProperty(e.key.toLowerCase())) {
            keys[e.key.toLowerCase()] = true;
            if(config.autoDrive) btnAutoDrive.click(); // Turn off auto drive if user types
        }
        if (keys.hasOwnProperty(e.key)) {
            keys[e.key] = true;
            if(config.autoDrive) btnAutoDrive.click();
        }
    });

    window.addEventListener('keyup', (e) => {
        if (keys.hasOwnProperty(e.key.toLowerCase())) keys[e.key.toLowerCase()] = false;
        if (keys.hasOwnProperty(e.key)) keys[e.key] = false;
    });

    window.addEventListener('resize', () => {
        const container = canvas.parentElement;
        if (container.clientWidth > 0 && container.clientHeight > 0) {
            canvas.width = container.clientWidth;
            canvas.height = container.clientHeight;
        }
    });
}

// Initialize everything
window.onload = () => {
    setupUI();
    initSimulation();
    requestAnimationFrame(gameLoop);
};
