const { goRed, goGreen, goYellow, goBlink } = require('../utils/traffic.commands');

const TrafficLightStatus = {
  RED: 0x00,
  YELLOW_TO_G: 0x01,
  GREEN: 0x02,
  YELLOW_TO_R: 0x03,
  BLINK: 0x04
};

class TrafficLight {
  constructor(id, x, y, status = TrafficLightStatus.RED, socket = null, durations = {}) {
    this.id = id;
    this.localization_x = x;
    this.localization_y = y;
    this.status = status;
    this.socket = socket;
    this.durations = durations;
  }

  isConnected() {
    return this.socket && this.socket.writable;
  }

  async goRed() {
    if (this.isConnected()) {
      this.status = TrafficLightStatus.RED;
      await goRed(this.socket, this.id);
    }
  }

  async goYellow() {
    if (this.isConnected()) {
      this.status = (this.status === TrafficLightStatus.RED)
        ? TrafficLightStatus.YELLOW_TO_G
        : TrafficLightStatus.YELLOW_TO_R;

      await goYellow(this.socket, this.id);
    }
  }

  async goGreen() {
    if (this.isConnected()) {
      this.status = TrafficLightStatus.GREEN;
      await goGreen(this.socket, this.id);
    }
  }

  async goBlink() {
    if (this.isConnected()) {
      this.status = TrafficLightStatus.BLINK;
      await goBlink(this.socket, this.id);
    }
  }

  changeStatus(newStatus) {
    this.status = newStatus;
  }

  getSocket() {
    return this.socket;
  }

  getDurations() {
    return this.durations;
  }
}

module.exports = { TrafficLight, TrafficLightStatus };
