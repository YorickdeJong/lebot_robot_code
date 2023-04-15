#!/usr/bin/env node

import rosnodejs from 'rosnodejs';
import { v4 as uuidv4 } from 'uuid';
const measuredData = rosnodejs.require('driver_bot_cpp').msg.distanceVelocity;
const recordId = uuidv4();
let measurementStarted = false;
let measurementStartingTime;

class PlotSubscriber {
    constructor() {
        this.posted = false; // Initialize flag to false
        this.userID = 1;
        this.assignmentNumber = 1;
        this.assignmentTitle = "Default Title";
    }

    init() {
        rosnodejs.initNode('connection_plot_to_app_node')
            .then(async (rosNode) => {
                this.userID = await rosNode.getParam('/connection_plot_to_app_node/userID');
                this.assignmentNumber = await rosNode.getParam('/connection_plot_to_app_node/assignmentNumber');
                this.assignmentTitle = await rosNode.getParam('/connection_plot_to_app_node/assignmentTitle');
                this.subjectTitle = await rosNode.getParam('/connection_plot_to_app_node/subjectTitle');
                const subscriber = rosNode.subscribe(
                    'deviceData',
                    measuredData,
                    this.onDataArrayMessage.bind(this),
                    { queueSize: 1, throttle_rate: 100 }
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
                    console.log(this.subjectTitle)
                }
                measurementStartingTime = measuredData.time[0]
                measurementStarted = true;
            }
            console.log(measuredData.distance)
            console.log(measuredData.motorNumber)
            
            const timeArray = measuredData.time;
            const distanceArrays = measuredData.distance.map(multiArray => multiArray.data);
            const velocityArrays = measuredData.velocity.map(multiArray => multiArray.data);
            const forceArrays = measuredData.force.map(multiArray => multiArray.data);
            const energyArrays = measuredData.energy.map(multiArray => multiArray.data);

            // Loop through the images in the array and send each one to the server
            try {
                return fetch(`http://10.7.191.125:3001/api/v1/measurement-results/${recordId}`, {
                    method: 'PUT',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        record_number: recordId,
                        distance: distanceArrays,
                        velocity: velocityArrays,
                        force: forceArrays,
                        energy: energyArrays,
                        time: timeArray,
                        motor_number: measuredData.motorNumber,
                        type: measuredData.type,
                        assignment_number: this.assignmentNumber, // Change this to the actual assignment number
                        user_id: this.userID, // Change this to the actual user profile ID
                        title: this.assignmentTitle, // Change this to the desired title
                        subject: this.subjectTitle,
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

plotSubscriber.init();


