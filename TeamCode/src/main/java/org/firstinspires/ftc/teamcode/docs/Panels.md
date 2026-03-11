# Panels, similar to sloth, is an amazing system
It is used to change system variables live and during runtime.

## Usage
In your code simply make a class annotated with the `@Configurable` trait. Then make any variables you wish to configure static.
```java
@Configurable
class TestingVariables {
    public static int loops = 0;
    public static double waitTime = 0.5;
    public static Color ball = Green;
}
```
A list of supported types and the docs for this can be found [here](https://ftcontrol.bylazar.com/docs/panels/configurables/).
For ease of use I copied the list of supported types
- Primitive types: int, double, boolean, etc.
- Enums
- Strings
- Arrays and Lists
- Maps (read-only unless exposed via a custom dashboard)
- Custom types (detected automatically)
- Generic types (via @GenericValue) (idk if this can be done in java) (probably) (but like idk when you would use this too)

When using panels for tuning, make sure you UPDATE THE ACTUAL VALUE IN YOUR CODE afterwards.
Panes makes only one way changes, so please please please update the original value.
