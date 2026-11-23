// A program written in the page runs here, in its own worker, so that Stop can always end it:
// the page terminates the worker, whatever the program is doing. The protocol is in runtime.js.
import { programEndpoint } from "./runtime.js";

const handle = programEndpoint((message) => self.postMessage(message));
self.onmessage = (event) => handle(event.data);
