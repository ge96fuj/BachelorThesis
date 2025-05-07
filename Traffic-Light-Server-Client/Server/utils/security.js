// utils/security.js
 const crypto = require("crypto");

function validateMessage(dataStr, options = {}) {
  const { secretKey, verifyTimeStamp = false, hashing = false, allowedDelay = 5 } = options;

  let parsed;
  try {
    parsed = JSON.parse(dataStr);
  } catch (err) {
    console.error("Invalid JSON received");
    return { valid: false, reason: "Invalid JSON" };
  }

  const receivedHmac = parsed.hmac;

  // Remove hmac field before verification
  const { hmac, ...msgWithoutHmac } = parsed;
  const jsonToHash = JSON.stringify(msgWithoutHmac);

  // --- HMAC check ---
  if (hashing) {
    if (!receivedHmac) {
      return { valid: false, reason: "Missing HMAC" };
    }

    const computedHmac = crypto
      .createHmac("sha256", secretKey)
      .update(jsonToHash)
      .digest("hex");

    if (
      !crypto.timingSafeEqual(
        Buffer.from(receivedHmac, "hex"),
        Buffer.from(computedHmac, "hex")
      )
    ) {
      return { valid: false, reason: "HMAC mismatch" };
    }
  }

  // --- Timestamp check ---
  if (verifyTimeStamp) {
    const now = Math.floor(Date.now() / 1000);
    const timestamp = parsed.timestamp;

    if (!timestamp) {
      return { valid: false, reason: "Missing timestamp" };
    }

    const delta = Math.abs(now - timestamp);
    if (delta > allowedDelay) {
      return { valid: false, reason: `Timestamp too old (${delta}s)` };
    }
  }

  return { valid: true, data: parsed };
}

module.exports = {
  validateMessage
};
