const express = require('express');
const path = require('path');

const app = express();

// Serve static files from the 'public' directory
app.use(express.static(path.join(__dirname, 'public')));

// Basic health check or API route (optional)
app.get('/api/status', (req, res) => {
  res.json({ status: 'ok', project: 'SLAM Playground' });
});

module.exports = app;
