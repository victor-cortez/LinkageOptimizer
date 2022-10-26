# Linkage Optimizer

This is a library for you to simulate a four bar linkage mechanism.

It is based on C++ with a python render script. The C++ part uses only standard library components.

# Usage

## Creating links, a header and a four bar mechanism

```cpp
#include "Link.h"
#include "CouplerHead.h"
#include "FourBarMechanism.h"

...

Link crank_link = Link(std::make_tuple(0.0, 0.0), std::make_tuple(0.0, 1.0), std_mass_linear_density);
Link coupler_link = Link(std::make_tuple(0.0, 1.0), std::make_tuple(0.2, 1.0), std_mass_linear_density);
Link output_link = Link(std::make_tuple(0.2, 1.0), std::make_tuple(1.0, 0.2), std_mass_linear_density);
CouplerHead header_link = CouplerHead(crank_link, output_link, std::make_tuple(0.0, 0.3), std::make_tuple(0.3, 0.7), std_mass_linear_density);
FourBarMechanism mechanism = FourBarMechanism(crank_link, coupler_link, output_link, header_link);
```

## Moving the mechanism

The rotate method is the only way of moving the mechanism now and it rotates the crank link by a given angle.
The method also updates the energy in each component using the previous position and the delta time interval given
to calculate the energy.

```cpp
mechanism.rotate(angle, 0.01);
```

## Creating a Field with button pairs

```cpp
#include "Field.h"

...

Field field = Field();
field.add_button_pair(std::make_tuple(0.0, 0.0), std::make_tuple(0.0, 1.0), std::make_tuple(0.0, 0.0), std::make_tuple(0.0, 1.0));
```

## Rendering the mechanism

Change this line in render.py to the path of the output file of the mechanism simulation

```python
data = pd.read_csv("example2ad.csv")
```

Run the script

```bash
python3 render.py
```

Check the output video in the videos folder
