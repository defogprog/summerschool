# Traffic light with button

__Content list__
1. About the program
2. Some features of the code
3. Video result

## About the program
Implemented a traffic light for pedestrians with a button. After pressing the button, the traffic light for cars will turn red, and for pedestrians - green.

## Some features of the code
_enum Green_state_ is used to create a variable that takes the value of a traffic light that should turn green (for cars / for people).

``` C
enum Green_state
{
	CARS, HUMANS
};
```

Function _flashing_ is needed in order to blink 3 times a certain diode.
``` C
void flashing(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
```

Function _HAL_GPIO_EXTI_Callback_ is called after pressing the button, writes a new value for the traffic light state.
``` C
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	state = HUMANS;
}
```

## Video result
You can watch the video result [here](https://user-images.githubusercontent.com/105476685/183343411-b330e1a1-9e67-4cf7-a901-285ef883cf97.mp4
)
