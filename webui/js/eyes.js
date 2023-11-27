const setEye = function (message) {
    const { linear, angular } = message;
    const { x } = linear;
    const { z } = angular;
    const scaledLinear = scaleNumber(x, -0.5, 0.5, 1, -1);
    const scaledRotation = scaleNumber(z, -1, 1, -1, 1);
    const rot = cartesianToPolar(scaledLinear, scaledRotation);
    var eye = $(".eye");
    eye.css({
      '-webkit-transform': 'rotate(' + rot + 'deg)',
      '-moz-transform': 'rotate(' + rot + 'deg)',
      '-ms-transform': 'rotate(' + rot + 'deg)',
      'transform': 'rotate(' + rot + 'deg)'
    });
  }
  
  const scaleNumber = function (value, fromMin, fromMax, toMin, toMax) {
    // Check if the value is within the source range
    if (value < fromMin || value > fromMax) {
      console.error("Input value is outside the source range.");
      return NaN;
    }
  
    // Calculate the percentage of the value within the source range
    const percentage = (value - fromMin) / (fromMax - fromMin);
  
    // Scale the percentage to the target range and return the result
    return toMin + percentage * (toMax - toMin);
  }
  
  const cartesianToPolar = function (x, y) {
    // Calculate the radius (r)
    const radius = Math.sqrt(x * x + y * y);
  
    // Calculate the angle (θ) in radians
    let angleRad = Math.atan2(y, x);
  
    // Convert the angle to degrees if needed
    const angleDeg = (angleRad * 180) / Math.PI;
  
    // Ensure the angle is within the range [0, 360)
    const positiveAngle = (angleDeg + 360) % 360;
  
    return positiveAngle;
  }
  

