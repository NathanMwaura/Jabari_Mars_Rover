<script>
  import { lastCommand } from '$lib/stores.js';
</script>

<div class="last-command-container">
  <strong>Last Command:</strong>
  <span>{$lastCommand || 'None'}</span>
</div>

<style>
  .last-command-container {
    padding: 10px;
    border: 1px solid #ccc;
    border-radius: 5px;
    margin-top: 10px;
  }
</style>