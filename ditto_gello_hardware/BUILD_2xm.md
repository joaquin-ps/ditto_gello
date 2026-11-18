# Building the Ditto GELLO leader, `2xm`

This guide builds the `2xm` arm: XM430 servos at joints 2 and 4, XC330 servos everywhere else. It follows the
[FACTR build guide](https://github.com/JasonJZLiu/FACTR_Hardware#building-instructions) and uses FACTR's screw
letters.

<img src="docs/build_2xm/finished.png" alt="The finished arm at its zero pose and at a working pose" width="760"/>

The pictures are drawn from the CAD model. Grey parts are already in place, the blue part is the one being added,
servos are black, and the screws of each step are pulled out along their own axes in the colour of their letter.
Link 5 is drawn as option A (`link5a.stl`); option B takes the same screws in the same places.

## What you need

| Item | Quantity | Note |
|---|---|---|
| Dynamixel XM430-W210-T | 2 | servos 2 and 4 |
| Dynamixel XC330-T288-T | 5 | servos 1, 3, 5, 6 and 7 |
| Idler set for XM430 (HN12-I101) | 2 | on servos 2 and 4 |
| Idler for XC330 (FPX330-H101) | 1 | on servo 6 |
| 3/4" bearing | 1 | in the base |
| U2D2, U2D2 Power Hub Board, 12 V supply, cables | 1 each | as FACTR |
| Printed parts | 7 | `stl/FACTR_base.stl`, and from `stl/2xm/`: `link1.stl` to `link6.stl`, with `link5a.stl` or `link5b.stl` as link 5 |

FACTR's [bill of materials](https://github.com/JasonJZLiu/FACTR_Hardware#bill-of-materials) has purchase links for
all of it. This arm has no gripper handle, so it needs one XC330 and the FPX330-S102 bracket fewer than FACTR.

You also need a thin driver for link 1's horn screws (see [Link 1](#link-1)). A magnetised bit helps there.

## Screw guide

<img src="docs/build_2xm/screws.png" alt="The five screws, A to E" width="620"/>

| Screw | Size | Quantity | Goes into |
|---|---|---|---|
| **A** | M2.5 × 8 mm | 12 | the body of an XM430 |
| **B** | M2 × 10 mm | 4 | servo 1, through the base (the only place it is used) |
| **C** | M2 × 8 mm, self-tapping | 28 | the body or the horn of an XC330 |
| **D** | M2 × 6 mm, self-tapping | 8 | the horn and idler of servo 6 |
| **E** | M2 × 6 mm | 14 | the horn or idler of an XM430 |

- The drawing shows lengths and threads to scale. The heads are drawn the same; yours will differ.
- FACTR's guide says screws C and D come with the XC330 servos. This arm uses 28 of C, four more than FACTR's arm,
  and has one XC330 fewer, so count yours before you start.
- C and D cut their own thread in plastic. Stop turning when the head seats.

## Step 1: Set up the servos

Do this before a servo goes into the arm. The servos all ship with ID 1 at 57600 bps, so connect them to the U2D2
one at a time and set each one with
[Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).
FACTR's ["Update Servo IDs"](https://github.com/JasonJZLiu/FACTR_Hardware#step-2-update-servo-ids) step shows where
to click.

| Setting | Address | Set to | Factory default |
|---|---|---|---|
| ID | 7 | the joint number, 1 to 7 (table below) | 1 |
| Baud Rate | 8 | 4000000 bps (value `6`) | 57600 bps (value `1`) |
| Return Delay Time | 9 | 0 | 500 µs (value `250`) |

| ID | Joint | Servo |
|---|---|---|
| 1 | J1 | XC330-T288-T |
| 2 | J2 | XM430-W210-T |
| 3 | J3 | XC330-T288-T |
| 4 | J4 | XM430-W210-T |
| 5 | J5 | XC330-T288-T |
| 6 | J6 | XC330-T288-T |
| 7 | J7 | XC330-T288-T |

- All three settings can only be changed while torque is off, and they are kept when power is removed.
- To find a servo again after this, scan at 4000000 bps.
- Write the ID on each servo, so the right one goes into each joint.

## Step 2: Print the parts

The [README](README.md#printing) has the print settings and how each part sits on the bed.

## Step 3: Assemble the arm

The arm is built from the base up. Two things apply to every step:

- **Servo at zero before a link goes on its horn.** Each link is cut so that the arm's zero pose is every servo's
  zero. Before you bolt a link to a horn, set that servo to the position your teleoperation software treats as zero.
  For most Dynamixel software that is the middle of the range, position 2048 in Dynamixel Wizard; check yours.
- **Groove on the top hole.** Every part that bolts to a horn has a small groove marking one screw hole. With the
  servo at zero, that hole goes on the horn's top hole: the one towards the servo's short end.

The pictures show the arm at its zero pose, which is folded: the forearm hangs down in front of the upper arm. If a
link sits on its servo as the picture shows, with the servo at zero, it is on the right way. `python view.py` and its
**Zero** button show the same pose in 3D.

Servos 1 and 2 plug into the Power Hub Board. Each later servo plugs into the one before it.

### Base

Press the **bearing** into the **base**, mount the **U2D2** and the **Power Hub Board** in the holder, and slide
**servo 1** in under the bearing with its horn up, centred in the bearing.

<img src="docs/build_2xm/base_1.png" alt="The base with the bearing, servo 1 and the electronics" width="640"/>

Turn the base over and fasten **servo 1** with four **screw B**. Connect servo 1 to the Power Hub Board.

<img src="docs/build_2xm/base_2.png" alt="Four screws B through the underside of the base into servo 1" width="560"/>

### Link 1

Set **servo 1** to zero. Push **link 1** fully into the bearing, with the groove on its floor plate over the horn hole
on the side away from the electronics. Fasten it to the horn with four **screw C**, one down each well.

<img src="docs/build_2xm/link1_1.png" alt="Link 1 in the bearing with four screws C above its wells" width="620"/>

The screws sit 58 mm down. The wells are cut for a driver that is, from its tip, 12 mm of thin bit, then 4 mm of
4 mm hex bit base, then a 7 mm shaft. A driver with a shorter thin end does not reach. Carry each screw down on a
magnetised bit: a dropped screw can fall flat in the cavity between the wells.

<img src="docs/build_2xm/link1_driver.png" alt="Link 1 from above, and cut through two horn screws with the driver in place" width="620"/>

Fit an **XM430 idler** to the back of **servo 2**. Slide servo 2 between link 1's ears with its long end down and its
horn on the side away from the electronics. Fasten it with four **screw A**, two through each ear. Connect servo 2
to the Power Hub Board.

<img src="docs/build_2xm/link1_2.png" alt="Servo 2 between the ears of link 1, two screws A on each side" width="560"/>

### Link 2

On the bench, put **servo 3** on **link 2**'s flange with its horn up and its short end over the front channel.
Fasten it from below with four **screw C**: two at the top of the open channel on the tower's front, put in from the
front with the driver lying in the channel, and two in the slots behind the tower.

<img src="docs/build_2xm/link2_1.png" alt="Servo 3 on link 2: two screws C in the front channel, two behind the tower" width="760"/>

Set **servo 2** to zero. Slide link 2's fork over servo 2, the slotted plate on the horn side, so that the slot
passes over the horn's centre. The groove on that plate goes on the horn's top hole, which points up. Fasten the
fork with three **screw E** on the horn side and four **screw E** on the idler side. Connect servo 3 to servo 2.

<img src="docs/build_2xm/link2_2.png" alt="Link 2's fork on servo 2: three screws E on the horn side, four on the idler side" width="760"/>

### Link 3

On the bench, stand **servo 4** on **link 3**'s ledge on its long end, with one side against the wall and the horn
facing as in the picture. Fasten it with eight **screw A**: four up through the ledge into the servo's end, and four
through the wall from behind. The picture looks at the part from below. Fit an **XM430 idler** to the back of
servo 4.

<img src="docs/build_2xm/link3_1.png" alt="Servo 4 on link 3: four screws A up through the ledge, four through the wall" width="640"/>

Set **servo 3** to zero. Put link 3 on servo 3's horn with the wall on the front side, above the tower's screw
channel. Fasten it with four **screw C** from above: three down the open bores behind the wall and one down the
groove in the wall's back. That groove is link 3's top-hole mark. Connect servo 4 to servo 3.

Once on the arm, servo 4's horn faces the electronics side of the base, opposite to servo 2's horn.

<img src="docs/build_2xm/link3_2.png" alt="Link 3 on servo 3's horn, four screws C from above" width="460"/>

### Link 4

On the bench, seat **servo 5** in **link 4**'s tray with its back on the tray floor, its horn out of the open side
and its long end towards the tray's tip. Fasten it with four **screw C** from the fork side: two straight into the
floor and two down the bores through the cross beam.

<img src="docs/build_2xm/link4_1.png" alt="Servo 5 in link 4's tray, four screws C from the fork side" width="560"/>

Set **servo 4** to zero. Slide link 4's fork over servo 4, the slotted plate on the horn side. The groove on that
plate goes on the horn's top hole, which points up. Fasten the fork with three **screw E** on the horn side and four
**screw E** on the idler side. Connect servo 5 to servo 4.

<img src="docs/build_2xm/link4_2.png" alt="Link 4's fork on servo 4: three screws E on the horn side, four on the idler side" width="760"/>

### Link 5

Set **servo 5** to zero. Put **link 5**'s flange on servo 5's horn, with the groove on the flange's edge on the
horn's top hole: the one towards the upper arm. Fasten it with four **screw C** from the open end of the forearm.
With option A that takes a long driver down the tube. With option B the screws sit in the open channels.

<img src="docs/build_2xm/link5_1.png" alt="Link 5 on servo 5's horn, four screws C driven up the tube" width="420"/>

Fit the **XC330 idler** to the back of **servo 6**. Slide servo 6's long end into the collar at the end of link 5,
with its horn on the same side as servo 2's horn. Fasten it with four **screw C**, two through each clamp wall.
Connect servo 6 to servo 5.

<img src="docs/build_2xm/link5_2.png" alt="Servo 6 in link 5's collar, two screws C on each side" width="460"/>

### Link 6

On the bench, seat **servo 7** in **link 6**'s tray, as servo 5 sits in link 4: back on the floor, horn out of the
open side, long end towards the tip. Fasten it with four **screw C** from the fork side.

<img src="docs/build_2xm/link6_1.png" alt="Servo 7 in link 6's tray, four screws C from the fork side" width="560"/>

Set **servo 6** to zero. Slide link 6's ears over servo 6. The ear with the groove goes on the horn side; the ear
with the centre hole goes on the idler. The groove goes on the horn's top hole, which points away from the forearm.
Fasten the ears with four **screw D** on the horn side and four **screw D** on the idler side. Connect servo 7 to
servo 6.

<img src="docs/build_2xm/link6_2.png" alt="Link 6's ears on servo 6: four screws D on the horn side, four on the idler side" width="760"/>

### Tool flange

Servo 7's horn is the end of the arm. Its four M2 holes are free for a handle or gripper trigger of your own.

## Screws by step

| Step | A | B | C | D | E |
|---|---|---|---|---|---|
| Base: servo 1 | | 4 | | | |
| Link 1 on servo 1's horn | | | 4 | | |
| Servo 2 in link 1 | 4 | | | | |
| Servo 3 on link 2 | | | 4 | | |
| Link 2 on servo 2 | | | | | 7 |
| Servo 4 on link 3 | 8 | | | | |
| Link 3 on servo 3's horn | | | 4 | | |
| Servo 5 in link 4 | | | 4 | | |
| Link 4 on servo 4 | | | | | 7 |
| Link 5 on servo 5's horn | | | 4 | | |
| Servo 6 in link 5 | | | 4 | | |
| Servo 7 in link 6 | | | 4 | | |
| Link 6 on servo 6 | | | | 8 | |
| **Total** | **12** | **4** | **28** | **8** | **14** |
