const crypto = require('crypto');
// Message format : [code][timestamp][hmac]
function buildAuthenticatedMessage(code) {
    const timestamp = Math.floor(Date.now() / 1000);
    const buf = Buffer.alloc(5);
    buf.writeUInt8(code, 0);
    buf.writeUInt32BE(timestamp, 1);

    const hmac = crypto.createHmac('sha256', global.SECRET_KEY)
        .update(buf)
        .digest();

    return Buffer.concat([buf, hmac]);
}

function sendCommand(socket, code, label, lightID = '') {
    if (!socket || socket.destroyed) {
        console.log(`Socket not available for ${lightID}`);
        global.stopLoop = true;
        return false;
    }

    message = Buffer.from([code]); 
    if (global.Hashing) {
         message = buildAuthenticatedMessage(code);
    }

    console.log(`${label} Sending secure (${code}) to ${lightID}`);
    socket.write(message);
    return true;
}

async function goRed(socket, lightID) {
    return sendCommand(socket, 0x21, "🔴 RED →", lightID);
}

async function goYellow(socket, lightID) {
    return sendCommand(socket, 0x23, "🟡 YELLOW →", lightID);
}

async function goGreen(socket, lightID) {
    return sendCommand(socket, 0x22, "🟢 GREEN →", lightID);
}

function goBlink(socket, lightID) {
    return sendCommand(socket, 0x25, "🟡🟡🟡🟡 BLINK 🟡🟡🟡🟡 →", lightID);
}




function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

module.exports = { goRed, goYellow, goGreen, goBlink, sleep, sendCommand };
