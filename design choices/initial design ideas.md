# Problems - transport
- Need to grab and transport blocks
- Need to raise blocks over the top of the box

## Design ideas
- Hold blocks at the bottom (basically just push them), figure out raising them over the box seperately from transport
- Grab blocks and put them up high on the robot, put them in the box by tilting platform holding th blocks or have a rolling floor that pushes them off
- Shoot the blocks when picking them up all the way to the box

### Ways to pick up the blocks up high
- Conveyer belt like structure
- Rotating arm - sort of like a bulldozer
- Rotating arm like bulldozer but this time has rotating wheels that pull the blocks through inside of the arm

# Problems - detection
- Detect blocks on field
- Detect box on field

## Additional specification of problem
- Blocks and even the box don't have any easy way to recognize them, both have only size specified and rest is ambiguos (color, pattern, ... are not specified)

## Design Ideas
- camera + trained AI categorization model
- 2 distance senzors (i.e. ultrasonic sensors) one up high, one lower -> one can see only box, one can see blocks and box
- Ignore detection of blocks and just go back and forth on the field

