#!/usr/bin/env node

import rosnodejs from 'rosnodejs';
const imagesMSG = rosnodejs.require('driver_bot_cpp').msg.images;

class PlotSubscriber {
    constructor() {
        this.posted = false; // Initialize flag to false
    }

    init() {
        rosnodejs.initNode('connection_plot_to_app_node')
            .then((rosNode) => {
                const subscriber = rosNode.subscribe(
                    'movement_plot',
                    imagesMSG,  // change to the correct message type
                    this.onImageArrayMessage.bind(this),
                    { queueSize: 1, throttle_rate: 100 }
                );

            })
            .catch((err) => {
                console.error(err);
            });

    }

    onImageArrayMessage(imagesMSG) {
        // Only post to the server once
        console.log(`MESSAGE RECEIVED: ${imagesMSG.data_type}}`);
        if (imagesMSG.data_type.length !== 0) {
            // Set flag to true to prevent multiple posts
            this.posted = true;

            // Loop through the images in the array and send each one to the server
            try {
                imagesMSG.images.forEach(async (image, index) => {
                    console.log(image.data.length)
                    return await fetch('http://172.20.10.2:3000/api/v1/images', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json'
                        },
                        body: JSON.stringify({
                            name: `${imagesMSG.data_type}plot${index}.png`, // Change this to the desired filename format
                            data: image.data,
                            mime_type: 'image/png',
                            size: image.data.length,
                            assignment_number: 1, // Change this to the actual assignment number
                            user_profile_id: 4, // Change this to the actual user profile ID
                            title: 'Beweging', // Change this to the desired title
                        })
                    })
                        .then(response => {
                            console.log(`Image ${index} uploaded successfully`);
                            rosnodejs.shutdown();
                        })
                        .catch(error => {
                            console.error(error);
                        });
                });
            }
            catch (err) {
                console.log(err);
                rosnodejs.shutdown();
            }
        }
    }
}
const plotSubscriber = new PlotSubscriber();


setTimeout(() => {
    plotSubscriber.init();
}, 1000);

