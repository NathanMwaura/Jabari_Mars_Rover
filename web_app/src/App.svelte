<script>
  import CameraFeed from './lib/components/CameraFeed.svelte';
  import PanTiltControl from './lib/components/PanTiltControl.svelte';
  import RobotControl from './lib/components/RobotControl.svelte';
  import SensorDashboard from './lib/components/SensorDashboard.svelte';
  import SystemMonitor from './lib/components/SystemMonitor.svelte';
  import LastCommand from './lib/components/LastCommand.svelte';
</script>

<main>
  <header>
    <div class="header-content">
      <div class="logo">
        <h1><i class="fas fa-robot"></i> ROS2 Robot Interface</h1>
      </div>
      <div class="system-status">
        <div class="connection-status" id="connectionStatus">
          <div class="status-indicator" id="statusIndicator"></div>
          <span id="statusText">Connecting...</span>
        </div>
        <div class="system-time" id="systemTime"></div>
      </div>
    </div>
  </header>

  <div class="container">
    <div class="main-layout">
      <section class="camera-section">
        <CameraFeed />
      </section>

      <section class="controls-section">
        <PanTiltControl />
        <RobotControl />
      </section>

      <section class="sensors-section">
        <SensorDashboard />
      </section>

      <section class="monitor-section">
        <SystemMonitor />
        <LastCommand />
      </section>
    </div>
  </div>
</main>

<style>
  /* Global styles can be placed here or in a separate CSS file */
</style>