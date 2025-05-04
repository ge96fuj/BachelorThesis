
function sendCommand(socket, code, lightID = '') {
    if (!socket || socket.destroyed) {
        console.log(`Socket is notalive ${lightID}`);
        global.stopLoop = true;
        return false;
    }
    console.log(` Send (${code}) to ${lightID}`);
    socket.write(Buffer.from([code]));
    return true;}

async function goRed(socket, lightID) {
    return sendCommand(socket, 0x21, lightID);}

async function goYellow(socket, lightID) {
    return sendCommand(socket, 0x23, lightID);}

async function goGreen(socket, lightID) {
    return sendCommand(socket, 0x22, lightID);}

function goBlink(socket, lightID) {
    return sendCommand(socket, 0x25, lightID);}


async function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
 }

module.exports = { goRed, goYellow, goGreen, goBlink, sleep };
