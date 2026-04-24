const puppeteer = require('puppeteer');
const http = require('http');
const app = require('./app');

// Increase timeout for Puppeteer tests (browser launch can take a few seconds)
jest.setTimeout(30000);

describe('UI Tests with Puppeteer', () => {
  let server;
  let browser;
  let page;
  const PORT = 3001; // Use a different port to avoid conflicts with development server

  beforeAll(async () => {
    // Start the local server
    server = http.createServer(app);
    await new Promise((resolve) => server.listen(PORT, resolve));

    // Launch a headless browser
    browser = await puppeteer.launch({
      headless: "new",
      args: ['--no-sandbox', '--disable-setuid-sandbox'] // Good defaults for test environments
    });
    page = await browser.newPage();
  });

  afterAll(async () => {
    // Clean up browser and server after tests
    if (browser) await browser.close();
    if (server) await new Promise((resolve) => server.close(resolve));
  });

  it('should load the page and display the correct title', async () => {
    await page.goto(`http://localhost:${PORT}`);
    const title = await page.title();
    expect(title).toBe('SLAM Learning Playground');
  });

  it('should render the simulation canvas', async () => {
    await page.goto(`http://localhost:${PORT}`);
    const canvas = await page.$('canvas#simCanvas');
    expect(canvas).not.toBeNull();
  });

  it('should toggle the Auto-Drive button when clicked', async () => {
    await page.goto(`http://localhost:${PORT}`);
    
    // Wait for the button to be available in the DOM
    const btnSelector = '#btnAutoDrive';
    await page.waitForSelector(btnSelector);
    
    // Get initial text
    let buttonText = await page.$eval(btnSelector, el => el.innerText);
    expect(buttonText).toContain('Auto-Drive: ON');

    // Click the button
    await page.click(btnSelector);

    // Get text after click
    buttonText = await page.$eval(btnSelector, el => el.innerText);
    expect(buttonText).toContain('Auto-Drive: OFF');
  });

  it('should update algorithm description when algorithm is changed', async () => {
    await page.goto(`http://localhost:${PORT}`);
    
    const selectSelector = '#algoSelect';
    const descSelector = '#algoDesc';
    
    await page.waitForSelector(selectSelector);
    
    // Select 'odometry'
    await page.select(selectSelector, 'odometry');
    let descText = await page.$eval(descSelector, el => el.innerText);
    expect(descText).toContain('Dead Reckoning');

    // Select 'mcl'
    await page.select(selectSelector, 'mcl');
    descText = await page.$eval(descSelector, el => el.innerText);
    expect(descText).toContain('Monte Carlo Localization');
  });
});
