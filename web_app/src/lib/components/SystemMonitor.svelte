<div class="card">
  <h3><i class="fas fa-desktop"></i> System Monitor</h3>
  <div class="system-monitor-container">
    <div class="monitor-header">
      <div class="connection-status connected">
        <div class="status-indicator"></div>
        <span>Demo Mode Active</span>
      </div>
      <div class="system-actions">
        <button class="btn btn-secondary" id="refreshBtn">
          <i class="fas fa-sync-alt"></i> Refresh
        </button>
        <button class="btn btn-danger" id="emergencyStopBtn">
          <i class="fas fa-exclamation-triangle"></i> E-Stop
        </button>
      </div>
    </div>

    <div class="system-stats-grid">
      <div class="stat-card">
        <div class="stat-icon">🖥️</div>
        <div class="stat-info">
          <div class="stat-label">System Uptime</div>
          <div class="stat-value" id="uptimeValue">2h 34m</div>
        </div>
      </div>
      <div class="stat-card">
        <div class="stat-icon">📊</div>
        <div class="stat-info">
          <div class="stat-label">Network Latency</div>
          <div class="stat-value" id="latencyValue">12ms</div>
        </div>
      </div>
      <div class="stat-card">
        <div class="stat-icon">🔄</div>
        <div class="stat-info">
          <div class="stat-label">Messages/sec</div>
          <div class="stat-value" id="messagesValue">247</div>
        </div>
      </div>
      <div class="stat-card">
        <div class="stat-icon">💽</div>
        <div class="stat-info">
          <div class="stat-label">Disk Usage</div>
          <div class="stat-value" id="diskValue">45%</div>
        </div>
      </div>
    </div>

    <div class="system-logs">
      <div class="logs-header">
        <h4>System Logs</h4>
        <div class="log-controls">
          <button class="btn" id="clearLogsBtn">Clear</button>
          <button class="btn" id="pauseLogsBtn">Pause</button>
          <select class="btn" id="logLevelSelect">
            <option value="all">All Levels</option>
            <option value="info">Info</option>
            <option value="warning">Warning</option>
            <option value="error">Error</option>
          </select>
        </div>
      </div>
      <div class="logs-container" id="logsContainer">
        <div class="log-entry log-info">
          <span class="log-timestamp">19:23:45</span>
          <span class="log-message">System initialized successfully</span>
        </div>
        <div class="log-entry log-info">
          <span class="log-timestamp">19:23:46</span>
          <span class="log-message">Camera feed started</span>
        </div>
        <div class="log-entry log-warning">
          <span class="log-timestamp">19:23:47</span>
          <span class="log-message">ROS2 connection not available - Demo mode active</span>
        </div>
        <div class="log-entry log-success">
          <span class="log-timestamp">19:23:48</span>
          <span class="log-message">Web interface ready</span>
        </div>
      </div>
    </div>
  </div>
</div>