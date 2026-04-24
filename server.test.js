const request = require('supertest');
const app = require('./app');

describe('Server Endpoints', () => {
  it('should serve the index.html file', async () => {
    const res = await request(app).get('/');
    expect(res.statusCode).toEqual(200);
    expect(res.text).toContain('<title>SLAM Learning Playground</title>');
  });

  it('should serve the styles.css file', async () => {
    const res = await request(app).get('/styles.css');
    expect(res.statusCode).toEqual(200);
    expect(res.header['content-type']).toContain('css');
  });

  it('should serve the script.js file', async () => {
    const res = await request(app).get('/script.js');
    expect(res.statusCode).toEqual(200);
    expect(res.header['content-type']).toContain('javascript');
  });

  it('should return 200 for the status API', async () => {
    const res = await request(app).get('/api/status');
    expect(res.statusCode).toEqual(200);
    expect(res.body).toEqual({ status: 'ok', project: 'SLAM Playground' });
  });

  it('should return 404 for non-existent routes', async () => {
    const res = await request(app).get('/non-existent');
    expect(res.statusCode).toEqual(404);
  });
});
