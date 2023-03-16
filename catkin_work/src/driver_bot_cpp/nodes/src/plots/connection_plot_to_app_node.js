#!/usr/bin/env node

import rosnodejs from 'rosnodejs';
import { v4 as uuidv4 } from 'uuid';
const measuredData = rosnodejs.require('driver_bot_cpp').msg.distanceVelocity;
const recordId = uuidv4();
let measurementStarted = false;
let measurementStartingTime;
let timeVelocityStartTime;
let timeEnergyStartTime;
let distanceForceStartDistace;

class PlotSubscriber {
    constructor() {
        this.posted = false; // Initialize flag to false
    }

    init() {
        rosnodejs.initNode('connection_plot_to_app_node')
            .then((rosNode) => {
                const subscriber = rosNode.subscribe(
                    'deviceData',
                    measuredData,  // change to the correct message type
                    this.onDataArrayMessage.bind(this),
                    { queueSize: 1, throttle_rate: 500 }
                );

            })
            .catch((err) => {
                console.error(err);
            });

    }

    onDataArrayMessage(measuredData) {
        if (measuredData.type.length !== 0) {
            // Set flag to true to prevent multiple posts
            if (!measurementStarted) {
                for (let i = 0; i < 15; i++) {
                    console.log("MEASUREMENT STARTED CONNECTION DATABASE ACTIVE");
                }
                measurementStartingTime = measuredData.time[0]
                timeVelocityStartTime = measuredData.timeVelocity[0]
                timeEnergyStartTime = measuredData.timeEnergy[0]
                distanceForceStartDistace = measuredData.distanceForce[0]
                measurementStarted = true;
            }
            // Loop through the images in the array and send each one to the server
            try {
                console.log(measuredData.timeVelocity)
                const time = measuredData.time.map(t => t - measurementStartingTime); //Adjust for measurement startup time
                return fetch(`http://172.20.10.2:3000/api/v1/measurement-results/${recordId}`, {
                    method: 'PUT',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        record_number: recordId,
                        distance: measuredData.distance,
                        velocity: measuredData.velocity,
                        force: measuredData.force,
                        energy: measuredData.energy,
                        time: time,
                        timeVelocity: measuredData.timeVelocity,
                        timeEnergy: measuredData.timeEnergy,
                        distanceForce: measuredData.distanceForce,
                        type: measuredData.type,
                        assignment_number: 1, // Change this to the actual assignment number
                        user_id: 4, // Change this to the actual user profile ID
                        title: 'Beweging', // Change this to the desired title
                    })
                })
                    .then(response => {
                        console.log(`DATA SEND SUCCESFULLY IN CONNECTION_PLOT_TO_APP_NODE: ${JSON.stringify(response)}`);
                    })
                    .catch(error => {
                        console.error(error);
                    });
            }
            catch (err) {
                console.log(err);
            }
        }
    }
}
const plotSubscriber = new PlotSubscriber();


// setTimeout(() => {
plotSubscriber.init();
// }, 1000);

