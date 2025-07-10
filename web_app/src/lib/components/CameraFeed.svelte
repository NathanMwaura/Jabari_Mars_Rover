<div class="card">
  <h3><i class="fas fa-video"></i> Camera Feed</h3>
  <div class="camera-container">
    <div class="camera-feed" id="cameraFeed">
      <div class="camera-placeholder">
        <i class="fas fa-camera" style="font-size: 3rem; margin-bottom: 1rem;"></i>
        <p>Camera feed will appear here</p>
        <p style="font-size: 0.9rem; color: var(--text-secondary);">Connecting to WebRTC...</p>
      </div>
    </div>
    <div class="camera-overlay">
      <span id="cameraStatus">Camera: Disconnected</span>
    </div>
  </div>
  <div class="camera-controls">
    <button class="btn" id="captureBtn">
      <i class="fas fa-camera"></i> Capture
    </button>
    <button class="btn" id="recordBtn">
      <i class="fas fa-video"></i> Record
    </button>
    <button class="btn" id="fullscreenBtn">
      <i class="fas fa-expand"></i> Fullscreen
    </button>
    <select class="btn" id="qualitySelect">
      <option value="high">High Quality</option>
      <option value="medium" selected>Medium Quality</option>
      <option value="low">Low Quality</option>
    </select>
  </div>
</div>