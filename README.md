# SLAM Learning Playground

A browser-based Simultaneous Localization and Mapping (SLAM) simulation and learning playground.

## Live Demo
Check out the live deployment here: [https://eitan101.github.io/slam/](https://eitan101.github.io/slam/)

## Features
- **Dead Reckoning (Odometry):** See how errors accumulate over time without reference points.
- **Monte Carlo Localization (MCL):** Localization within a known map using a particle filter.
- **FastSLAM:** Simultaneous localization and mapping in an unknown environment.

## Local Development

### Prerequisites
- [Node.js](https://nodejs.org/) installed on your machine.

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/eitan101/slam.git
   cd slam
   ```
2. Install dependencies:
   ```bash
   npm install
   ```

### Running the App
Start the local Express server:
```bash
npm start
```
The app will be available at [http://localhost:3000](http://localhost:3000).

### Testing
This project uses **Jest** for API testing and **Puppeteer** for End-to-End (E2E) UI testing.

To run the full test suite:
```bash
npm test
```

## Deployment
The frontend (`public/` directory) is configured to deploy to GitHub Pages.

To deploy the latest changes:
```bash
npm run deploy
```
*(Note: `npm run deploy` will automatically run the test suite to ensure everything passes before deploying).*
