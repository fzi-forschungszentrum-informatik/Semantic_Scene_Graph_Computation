<template>
  <div ref="graph" class="graph" />
</template>

<script setup lang="ts">
import { onMounted, reactive, ref, watch } from "vue";
import { Network, parseDOTNetwork } from "vis-network/standalone";

const props = defineProps({
  dotString: { type: String, required: true },
});

const graph = ref<HTMLDivElement | null>(null);

// Const network options
const layout = {
  improvedLayout: true,
  hierarchical: {
    enabled: false,
  },
};
const manipulation = {
  enabled: false,
  initiallyActive: false,
  addNode: true,
  addEdge: true,
  editEdge: true,
  deleteNode: true,
  deleteEdge: true,
};
const interaction = {
  dragNodes: true,
  dragView: true,
  hideEdgesOnDrag: false,
  hideEdgesOnZoom: false,
  hideNodesOnDrag: false,
  hover: true,
  hoverConnectedEdges: true,
  keyboard: {
    enabled: false,
    speed: { x: 10, y: 10, zoom: 0.02 },
    bindToWindow: true,
    autoFocus: true,
  },
  multiselect: false,
  navigationButtons: true,
  selectable: true,
  selectConnectedEdges: true,
  tooltipDelay: 300,
  zoomSpeed: 1,
  zoomView: true,
};
const physics = {
  enabled: true,
  barnesHut: {
    theta: 0.5,
    gravitationalConstant: -2000,
    centralGravity: 0.3,
    springLength: 200,
    springConstant: 0.04,
    damping: 0.09,
    avoidOverlap: 0,
  },
  forceAtlas2Based: {
    theta: 0.5,
    gravitationalConstant: -50,
    centralGravity: 0.01,
    springConstant: 0.08,
    springLength: 100,
    damping: 0.4,
    avoidOverlap: 0,
  },
  repulsion: {
    centralGravity: 0.2,
    springLength: 200,
    springConstant: 0.05,
    nodeDistance: 100,
    damping: 0.09,
  },
  hierarchicalRepulsion: {
    centralGravity: 0.0,
    springLength: 100,
    springConstant: 0.01,
    nodeDistance: 120,
    damping: 0.09,
    avoidOverlap: 0,
  },
  maxVelocity: 50,
  minVelocity: 0.1,
  solver: "barnesHut",
  stabilization: {
    enabled: true,
    iterations: 1000,
    updateInterval: 100,
    onlyDynamicEdges: false,
    fit: true,
  },
  timestep: 0.5,
  adaptiveTimestep: true,
  wind: { x: 0, y: 0 },
};
const coptions = { layout, manipulation, interaction, physics };

const state = reactive({
  data: {
    nodes: [],
    edges: [],
  },
  options: {},
});
var network;

function redraw() {
  var parsedData = parseDOTNetwork(props.dotString);

  state.data = {
    nodes: parsedData.nodes,
    edges: parsedData.edges,
  };

  state.options = Object.assign(parsedData.options, coptions);

  network.setData(state.data);
  network.setOptions(state.options);
  network.redraw();
}

watch(
  () => props.dotString,
  (first, second) => {
    redraw();
  }
);

onMounted(() => {
  network = new Network(graph.value, state.data, state.options);
  redraw();
});
</script>

<style scoped>
.graph {
  height: 80%;
}
</style>