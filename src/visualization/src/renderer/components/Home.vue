<template>
  <div id="home">
    <div class="toolbar" style="flex-wrap: nowrap" justify="end">
      <button v-on:click="input.click()">load files</button>
      <input
        v-show="false"
        ref="input"
        type="file"
        accept=".dot"
        @change="handleFileUpload()"
        multiple
      />

      <button v-if="state.total > 0" @click="previous">previous</button>
      <button v-if="state.total > 0" @click="next">next</button>
      <p v-if="state.total > 0">
        Display Graph: {{ state.index + 1 }}/{{ state.total }}
      </p>
    </div>
    <Transition>
      <vis-network
        v-if="state.dotString.length > 0"
        :dotString="state.dotString"
      ></vis-network>
    </Transition>
    <p v-if="state.total > 0">File: {{ state.fileName }}</p>
  </div>
</template>

<script setup lang="ts">
import { reactive, ref } from "vue";
import VisNetwork from "./VisNetwork.vue";

// Component state
interface HomeState {
  index: number;
  total: number;
  dotString: string;
  fileName: string;
}

const state: HomeState = reactive({
  index: 0,
  total: 0,
  dotString: "",
  fileName: "",
});

// Called when the state.index has changed
function updateState() {
  // trigger reading dot file
  reader.readAsText(input.value.files[state.index]);
  // set new file name
  state.fileName = input.value.files[state.index].name;
}

// Handle files
const reader = new FileReader();
reader.onload = (e) => {
  const fileContent = reader.result.toString();
  let tmp = fileContent;

  // add double quotes for label value
  tmp = tmp.replace(/(\s+label=)([\w\d]+)/g, '$1"$2"');

  // remove double quotes in "projection_group"
  tmp = tmp.replace(/(\s+projection_identity=)"([\w\d\s-:,\.]+)"/g, "$1$2");

  // fix nodes
  tmp = tmp.replace(
    /(\s+\d+\s\[[\r\n\t\s]*label="[\w\d​]+"[\r\n\t\s]*)(id[A-Za-z0-9=_:,\{\}\-\.\s\r\n\t]+)(\];)/g,
    '$1title="$2"$3'
  );

  // fix edges
  tmp = tmp.replace(
    /(\s+\d+\s->\s\d+\s\[[\r\n\t\s\w\d="]*)(type[A-Za-z0-9=_:\-\.\s\r\n\t]+)(\];)/g,
    '$1title="$2"$3'
  );

  state.dotString = tmp;

  // console.log("dotString:", state.dotString);
};

const input = ref<HTMLInputElement | null>(null);
const handleFileUpload = async () => {
  state.index = 0;
  state.total = input.value.files.length;

  updateState();
};

// Handle graph counter
function previous() {
  state.index = (state.index - 1) % state.total;

  // Fix neg values from modulo
  if (state.index < 0) {
    state.index += state.total;
  }

  updateState();
}
function next() {
  state.index = (state.index + 1) % state.total;

  updateState();
}
</script>

<style scoped>
#home {
  font-family: "Avenir", Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  display: flex;
  flex-direction: column;
  width: 100%;
  height: 100%;
}

h1,
h2 {
  font-weight: normal;
}

ul {
  list-style-type: none;
  padding: 0;
}

li {
  display: inline-block;
  margin: 0 10px;
}

a {
  color: #42b983;
}
</style>
