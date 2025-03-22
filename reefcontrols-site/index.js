import { NT4_Client } from "./NT4.js";

const ntClient = new NT4_Client(
    window.location.hostname,
    "Gamma2025",
    () => {
        // Topic announce
    },
    () => {
        // Topic unannounce
    },
    (topic, _, value) => {
        // New data
    },
    () => {
        // Connected
        document.body.style.backgroundColor = "white";
    },
    () => {
        // Disconnected
        document.body.style.backgroundColor = "red";
    }
    );

window.addEventListener("load", () => {
    ntClient.publishTopic("/DriveState/counter", "int");
    ntClient.connect();

    console.log("setup!");
});

let counter = 0;

document.getElementById("counter").addEventListener("click", () => {
    console.log(`clicked! ${counter}`);
    counter++;

    ntClient.addSample("/DriveState/counter", counter);
});