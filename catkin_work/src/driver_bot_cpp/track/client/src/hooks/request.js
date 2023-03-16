const API_URL = 'http://172.20.10.2:9000';  //changes with each internet connection!

// Load launches, sort by flight number, and return as JSON.
async function httpGetResults() {
    const response = await fetch(`${API_URL}/LeaderBoard`);
    const fetchedLeaderBoard = await response.json();
    return fetchedLeaderBoard;
}

// Submit given launch data to launch system.
async function httpSubmitResult(result) {
    //need to catch if succesful -> otherwise its false
    try {
        return await fetch(`${API_URL}/LeaderBoard`, { //fetch function defaults to get, so we set method to post
            method: "post",
            headers: { //need to specify what data type we are sending into the body
                "Content-Type": "application/json",
            },
            body: JSON.stringify(result), //convert obj to string
        }); //fetch function defaults to the get method
    }
    catch (err) {
        console.log("httpSubmitRequest failed");
        console.log(err);
        return {
            ok: false,
        };
    }
}

// Submit given launch data to launch system.
async function httpSubmitPowerUp(result) {
    //need to catch if succesful -> otherwise its false
    try {
        return await fetch(`${API_URL}/PowerUp`, { //fetch function defaults to get, so we set method to post
            method: "post",
            headers: { //need to specify what data type we are sending into the body
                "Content-Type": "application/json",
            },
            body: JSON.stringify(result), //convert obj to string
        }); //fetch function defaults to the get method
    }
    catch (err) {
        console.log("httpSubmitPowerUp failed");
        console.log(err);
        return {
            ok: false,
        };
    }
}


export { httpGetResults, httpSubmitResult, httpSubmitPowerUp };