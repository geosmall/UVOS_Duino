geosmall: How do I convert between a uvs_gpio_pin and a GPIO object?

GitHub Copilot: To convert between a `uvs_gpio_pin` and a `GPIO` object, you can use the conversion operator defined in the `Pin` struct within the `uvos` namespace. Here is how you can do it:

### Converting `uvs_gpio_pin` to `GPIO::Pin`

You can use the constructor of `GPIO::Pin` to create a `Pin` object from a `uvs_gpio_pin` object.

```cpp
// Assuming you have a uvs_gpio_pin object
uvs_gpio_pin oldPin = {UVS_GPIOA, 5};

// Convert to GPIO::Pin
uvos::Pin newPin(static_cast<uvos::GPIOPort>(oldPin.port), oldPin.pin);
```

### Converting `GPIO::Pin` to `uvs_gpio_pin`

You can use the conversion operator defined in the `Pin` struct to convert a `GPIO::Pin` object to a `uvs_gpio_pin` object.

```cpp
// Assuming you have a GPIO::Pin object
uvos::Pin newPin(uvos::PORTA, 5);

// Convert to uvs_gpio_pin
uvs_gpio_pin oldPin = static_cast<uvs_gpio_pin>(newPin);
```

### Example Usage in Code

Here is an example of how you might use these conversions in your code:

```cpp
#include "gpio.h"
#include "uvos_core.h"

void exampleConversion()
{
    // Create a uvs_gpio_pin object
    uvs_gpio_pin oldPin = {UVS_GPIOA, 5};

    // Convert to GPIO::Pin
    uvos::Pin newPin(static_cast<uvos::GPIOPort>(oldPin.port), oldPin.pin);

    // Now you can use newPin with the GPIO class
    uvos::GPIO gpio;
    uvos::GPIO::Config config;
    config.pin = newPin;
    config.mode = uvos::GPIO::Mode::OUTPUT;
    config.pull = uvos::GPIO::Pull::NOPULL;
    config.speed = uvos::GPIO::Speed::LOW;
    gpio.Init(config);

    // Convert back to uvs_gpio_pin if needed
    uvs_gpio_pin convertedBackPin = static_cast<uvs_gpio_pin>(newPin);
}
```

This example demonstrates how to convert between the old `uvs_gpio_pin` type and the new `GPIO::Pin` type, allowing you to use the new `GPIO` class with the old pin definitions.
