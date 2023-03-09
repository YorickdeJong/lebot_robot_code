#!/usr/bin/env node
const rosnodejs = require("rosnodejs");

import {
    useSubmit,
    usePowerUpSubmit
} from "../hooks/useResult.js";

const robotId = CreateRobotID();
const startTime = new Date();
var countRound = 0;
var countPowerUp = 0;
var totalTime = 0;

function subRound() {

    //Get round number and time
    rosnodejs.initNode('jsNode', { onTheFly: true })
        .then(nh => {
            const callbackRound = function (msg) {
                if (Object.keys(msg).length !== 0) {
                    const key = robotId * 100 + countRound;
                    const round = msg.data;
                    const roundTime = toSeconds();
                    totalTime += roundTime;

                    //Submit Launch
                    const submitResult = useSubmit(key, robotId, round, roundTime, totalTime);
                    submitResult();
                    countRound += 1;
                }
                else {
                    console.log("Round not received in WEBSITE NODE");
                }
            }

            const callbackPowerUp = function (msg) {
                if (Object.keys(msg).length !== 0) {
                    const key = robotId * 1000 + countPowerUp;
                    const powerUp = msg.data;

                    //Submit Launch
                    const submitPowerUp = usePowerUpSubmit(key, robotId, powerUp);
                    submitPowerUp();
                    countPowerUp += 1;
                }
                else {
                    console.log("Power Up not received in WEBSITE NODE");
                }
            }

            const subRound = nh.subscribe('qrRound', 'std_msgs/Int32', callbackRound);
            //const subCallbackPowerUp = nh.subscribe('qrPowerPublisher', 'std_msgs/Int8', callbackPowerUp);
        })
        .catch(err => {
            console.log(err);
        });

}

function toSeconds() {
    const time = new Date(); //only set date if robot send a round
    return Math.floor((time.getTime() - startTime.getTime()) / 1000);

}

function CreateRobotID() { //want to call this function once per player 
    //create random number
    //while number not in database repeat
    const min = Math.ceil(1);
    const max = Math.floor(100);
    return Math.floor(Math.random() * (max - min) + min);
}

// Start reading from stdin so we don't exit.
process.stdin.resume();



// catch ctrl+c event and exit normally
process.on('SIGINT', function () {
    console.log('Ctrl+C was keyed in, make you webhook call here');
    process.exit(0);
});

setTimeout(() => {
    subRound();
}, 1000);

