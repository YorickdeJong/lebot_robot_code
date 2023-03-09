import axios from 'axios';
// Do I want to make this a ROS env? -> send images to ROS node and then send those image to data base
const folderElement = document.getElementById('folder'); // Assumes the folder contents are displayed in a DOM element with ID "folder"

const observer = new MutationObserver((mutationsList, observer) => {
    const newFolderContents = Array.from(folderElement.children).map(child => child.textContent.trim());

    if (!arraysEqual(newFolderContents, folderContents)) {
        folderContents = newFolderContents;

        // Send the encoded image data to the server
        encodeImage('/src/driver_bot_cpp/nodes/Plots/images' + folderContents[0]) // Assumes the first item in the folder is the image file
            .then(({ encodedImage, imageSize }) => {
                axios.post('http://172.20.10.2:3000/api/v1/images', {
                    name: folderContents[0],
                    data: encodedImage,
                    mime_type: 'image/png',
                    size: imageSize,
                    assignment_number: assignment_number,
                    user_profile_id: user_profile_id,
                    title: title
                })
                    .then(response => {
                        console.log('Image uploaded successfully');
                    })
                    .catch(error => console.error(error));
            })
            .catch(error => console.error(error));
    }
});

observer.observe(folderElement, { childList: true });

let folderContents = Array.from(folderElement.children).map(child => child.textContent.trim());

async function encodeImage(imageUrl) {
    const response = await fetch(imageUrl);
    const buffer = await response.arrayBuffer();
    const encodedImage = btoa(new Uint8Array(buffer).reduce((data, byte) => data + String.fromCharCode(byte), ''));
    const imageSize = encodedImage.length * 0.75; // The encoded image size is approximately 1.33 times the original size

    return { encodedImage, imageSize };
}

function arraysEqual(a, b) {
    if (a === b) return true;
    if (a == null || b == null) return false;
    if (a.length !== b.length) return false;

    for (let i = 0; i < a.length; i++) {
        if (a[i] !== b[i]) return false;
    }

    return true;
}