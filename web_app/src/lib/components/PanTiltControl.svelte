<div class="card">
  <h3><i class="fas fa-gamepad"></i> Pan-Tilt Control</h3>
  <div class="pan-tilt-controls">
    <div class="joystick-container" id="joystickContainer">
      <div class="joystick" id="joystick"></div>
    </div>
    <div class="preset-positions">
      <button class="btn" data-position="home">
        <i class="fas fa-home"></i> Home
      </button>
      <button class="btn" data-position="left">
        <i class="fas fa-arrow-left"></i> Left
      </button>
      <button class="btn" data-position="right">
        <i class="fas fa-arrow-right"></i> Right
      </button>
    </div>
    <div class="position-values">
      <div class="value-display">
        <label for="panValue">Pan:</label>
        <output id="panValue">0°</output>
      </div>
      <div class="value-display">
        <label for="tiltValue">Tilt:</label>
        <output id="tiltValue">0°</output>
      </div>
    </div>
  </div>
</div>