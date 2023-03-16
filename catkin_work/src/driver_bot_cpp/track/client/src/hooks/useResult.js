import {
    httpGetResults,
    httpSubmitResult,
    httpSubmitPowerUp
} from './request.js';

function useSubmit(key, robotId, round, roundTime, totalTime) {
    //check if data is none empty -> otherwise do not send
    const submitResult = async () => {
        const response = await httpSubmitResult({
            key,
            robotId,
            round,
            roundTime,
            totalTime
        });

        const success = response.ok; //.ok tests if an expression is true or not
        if (success) {
            console.log('updated results')
        }
        else {
            console.log('failed to update result');
        }

        console.log(`RobotID LENGHT: ${robotId}
        Round length: ${round}
        time length: ${roundTime}
        time length: ${totalTime}`);


    }
    return submitResult;

}

function usePowerUpSubmit(key, powerUp) {
    //check if data is none empty -> otherwise do not send
    const submitPowerUp = async () => {
        const response = await httpSubmitPowerUp({
            key,
            robotId,
            powerUp
        });

        // TODO: Set success based on response.
        const success = response.ok; //.ok tests if an expression is true or not
        if (success) {
            console.log('updated results')
        }
        else {
            console.log('failed to update result');
        }

        console.log(`key: ${key}
        robotId: ${robotId}
        powerUp: ${powerUp}`);

    }
    return submitPowerUp;
}

export { useSubmit, usePowerUpSubmit }
