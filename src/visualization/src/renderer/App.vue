<template>
  <div id="app">
    <home></home>
  </div>
</template>

<script lang="ts">
import { defineComponent } from "vue";
import Home from "./components/Home.vue";
import { ipcRenderer } from "./electron";

export default defineComponent({
  name: "app",
  components: {
    Home,
  },
  setup() {
    ipcRenderer.send("message", "Hello from App.vue!");
  },
});
</script>

<style scoped>
#app {
  height: 100vh;
}
</style>
