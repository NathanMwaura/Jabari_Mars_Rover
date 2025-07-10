# JABARI Mars Rover Web Interface - Migration to Svelte+Vite

This document outlines the process for migrating the JABARI Mars Rover's web interface from a traditional HTML, CSS, and vanilla JavaScript structure to a modern, high-performance stack using Svelte and Vite.

## Process Map: Migration to Svelte+Vite

This process map details the steps to transition the existing web application to the new, recommended architecture.

### **Phase 1: Project Initialization & Cleanup**

1.  **Clean Existing Project:**
    *   [x] Remove `node_modules` and `package-lock.json`.
    *   [ ] Delete legacy JavaScript files (`script.js`, `public/app.js`, etc.).
    *   [ ] Move static assets (`styles.css`, images, icons) into a new `src/assets` directory.

2.  **Initialize Vite + Svelte Project:**
    *   [ ] Set up a new Vite project with the Svelte template. This will create the necessary configuration files (`vite.config.js`, `svelte.config.js`).
    *   [ ] Update `package.json` with new dependencies (`vite`, `svelte`, `@sveltejs/vite-plugin-svelte`) and scripts (`dev`, `build`, `preview`).

3.  **Install Dependencies:**
    *   [ ] Run `npm install` to install the new Vite and Svelte-related packages, along with other necessary libraries like `socket.io-client` and `nipplejs`.

### **Phase 2: Componentization**

1.  **Create Svelte Components:**
    *   [ ] Convert the major sections of the `index.html` file into individual `.svelte` components within `src/lib/components/`.
        *   `CameraFeed.svelte`
        *   `PanTiltControl.svelte`
        *   `RobotControl.svelte`
        *   `SensorDashboard.svelte`
        *   `SystemMonitor.svelte`
    *   [ ] The main `index.html` will be simplified to a basic shell with a `<div id="app"></div>` where the Svelte application will be mounted.

2.  **Style Components:**
    *   [ ] Migrate the global `styles.css` to be imported into the main Svelte component or directly within individual components' `<style>` blocks for scoped styling.

### **Phase 3: State Management & Communication**

1.  **Implement Centralized State (Svelte Stores):**
    *   [ ] Create a `src/lib/stores.js` file.
    *   [ ] Define writable stores to manage the robot's state, such as `robotStatus`, `sensorData`, and `connectionState`.

2.  **Develop Communication Services:**
    *   [ ] Create a `src/lib/services/ros-websocket.js` module. This service will be responsible for:
        *   Establishing and maintaining the WebSocket connection to the ROS2 `web_bridge_node`.
        *   Listening for incoming messages and updating the Svelte stores accordingly.
        *   Providing functions to send commands to the robot, which can be called from any Svelte component.
    *   [ ] Create a `src/lib/services/webrtc-client.js` module to handle the WebRTC connection with the `web_video_server`.

### **Phase 4: Integration & Finalization**

1.  **Assemble the Main Application:**
    *   [ ] In `src/App.svelte` (the main component), import and arrange the individual UI components to reconstruct the dashboard layout.
    *   [ ] Initialize the `ros-websocket.js` and `webrtc-client.js` services when the application loads.

2.  **Testing and Validation:**
    *   [ ] Use the Vite development server (`npm run dev`) to test the new application.
    *   [ ] Verify that all components correctly subscribe to the stores and update in real-time.
    *   [ ] Ensure that commands are sent correctly via the WebSocket service and that the video stream is displayed via WebRTC.

3.  **Production Build:**
    *   [ ] Run `npm run build` to create an optimized, production-ready build in the `dist` directory.
    *   [ ] The Node.js/Express server in the `web_interface_pkg` will be updated to serve the static files from this `dist` directory.

This structured migration will result in a more maintainable, scalable, and performant web interface, fully realizing the advanced architecture planned for the JABARI Mars Rover.