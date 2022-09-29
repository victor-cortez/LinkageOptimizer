import pylinkage as pl


def gen_4_bar_linkage():
    crank = pl.Crank(0, 1, joint0=(0, 0), angle=0.31, distance=1)
    pin = pl.Pivot(3, 2, joint0=crank, joint1=(3, 0), distance0=3, distance1=1)
    crank2 = pl.Crank(0, 1, joint0=crank, angle=0.31, distance=1)
    pin2 = pl.Pivot(3, 2, joint0=crank, joint1=crank2,
                    distance0=3, distance1=1)
    my_linkage = pl.Linkage(joints=(crank, pin))
    crank.name = "B"
    pin.name = "C"
    # Linkage can also have names
    my_linkage.name = "Four-bar linkage"


# Main motor
crank = pl.Crank(0, 1, joint0=(0, 0), angle=.31, distance=1, name="B")
# Close the loop
pin = pl.Pivot(3, 2, joint0=crank, joint1=(3, 0),
               distance0=3, distance1=1, name="C")

crank2 = pl.Crank(0, 2.5, joint0=crank, angle=0.0, distance=1.5, name="B2")
pin2 = pl.Pivot(3, 3, joint0=crank2, joint1=pin,
                distance0=3, distance1=1.5, name="C2")
fix = pl.Fixed(joint0=crank, joint1=pin2)
pin3 = pl.Pivot(3, 3, joint0=crank, joint1=pin2,
                distance0=3, distance1=1.5, name="C3")
fix1 = pl.Fixed(joint0=crank, joint1=(0, 2))
fix2 = pl.Fixed(joint0=fix1, joint1=pin)
# Linkage definition
my_linkage = pl.Linkage(
    joints=(crank, pin, fix1, fix2),
    order=(crank, pin, fix1, fix2),
    name="My four-bar linkage"
)

# Visualization
pl.show_linkage(my_linkage)
