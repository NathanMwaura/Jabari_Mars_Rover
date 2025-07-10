<div class="card" style="margin-top: 20px;">
  <h3><i class="fas fa-arrows-alt"></i> Robot Movement</h3>
  <div class="robot-controls">
    <div class="movement-controls">
      <button class="btn" id="turnLeftBtn">
        <i class="fas fa-undo"></i>
      </button>
      <button class="btn" id="forwardBtn">
        <i class="fas fa-arrow-up"></i>
      </button>
      <button class="btn" id="turnRightBtn">
        <i class="fas fa-redo"></i>
      </button>
      <button class="btn btn-danger" id="stopBtn">
        <i class="fas fa-stop"></i>
      </button>
      <button class="btn" id="backwardBtn">
        <i class="fas fa-arrow-down"></i>
      </button>
    </div>
    <div class="speed-control">
      <label for="speedSlider">Speed Control:</label>
      <input type="range" id="speedSlider" class="speed-slider" min="0" max="100" value="50">
      <div class="speed-display">
        <span>Speed: <span id="speedValue">50</span>%</span>
      </div>
    </div>
    <div class="mode-selector" style="margin-top: 15px;">
      <label for="controlMode">Control Mode:</label>
      <select id="controlMode" class="btn">
        <option value="manual">Manual</option>
        <option value="auto">Auto-Navigate</option>
        <option value="follow">Follow Mode</option>
      </select>
    </div>
  </div>
</div>