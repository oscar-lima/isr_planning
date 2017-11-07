GPSR General Purpose service robot
===

format:

Move to the LOCATION, move to the LOCATION, and move to the LOCATION.
Move to the LOCATION, move to the LOCATION and leave the apartment.
Move to the LOCATION, grasp the ITEM, and bring it to me.
Move to the LOCATION, grasp the ITEM, and bring it to the LOCATION.
Move to the LOCATION, grasp the ITEM, and put it in the trash bin.
Move to the LOCATION, find a person, and introduce yourself.
Move to the LOCATION, find a person, and guide it to the exit.

commands:

move, grasp, find person, introduce, guide, bring

Examples
===

1a. GPSR command generator Sentence (cat 1)

move to the sideboard, find a person, and introduce yourself

1b. Current nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: []
  -
    recognized_action: other
    slot: []
  -
    recognized_action: find
    slot: ['person is an object']
  -
    recognized_action: take
    slot: ['yourself is a destination']

1c. Desired nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['sideboard']
  -
    recognized_action: find
    slot: ['person']
  -
    recognized_action: introduce
    slot: ['myself']

-------------------------------

2a. GPSR command generator Sentence (cat 1)

move to the hallway table, find a person, and guide it to the exit

2b. Current nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['hallway table is a destination']
  -
    recognized_action: find
    slot: ['person is an object']
  -
    recognized_action: guide
    slot: ['it is an object', 'exit is a destination']

2c. Desired nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['hallway table']
  -
    recognized_action: find
    slot: ['person']
  -
    recognized_action: guide
    slot: ['person', 'exit']

------------------------------

3a. GPSR command generator Sentence (cat 1)

move to the dresser, move to the stove, and move to the sideboard

3b. Current nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['dresser is a destination']
  -
    recognized_action: go
    slot: ['stove is a destination']
  -
    recognized_action: go
    slot: ['sideboard is a destination']

3c. Desired nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['dresser']
  -
    recognized_action: go
    slot: ['stove']
  -
    recognized_action: go
    slot: ['sideboard']

------------------------------

4a. GPSR command generator Sentence (cat 1)

move to the sideboard, move to the hallway table and leave the apartment

4b. Current nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['sideboard is a destination']
  -
    recognized_action: go
    slot: ['hallway table is a destination']
  -
    recognized_action: other
    slot: ['hallway table is a destination']

4c. Desired nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['sideboard']
  -
    recognized_action: go
    slot: ['hallway table']
  -
    recognized_action: go
    slot: ['outside']

------------------------------

5a. GPSR command generator Sentence (cat 1)

move to the kitchen counter, grasp the Energy drink and bring it to the dinner table

5b. Current nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['kitchen is a destination', 'counter is a destination']
  -
    recognized_action: grasp
    slot: []
  -
    recognized_action: take
    slot: ['it is an object', 'dinner table is a destination']

5c. Desired nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['kitchen counter']
  -
    recognized_action: grasp
    slot: ['Energy drink']
  -
    recognized_action: bring
    slot: ['energy drink','dinner table']

------------------------------

6a. GPSR command generator Sentence (cat 1)

move to the kitchen counter, grasp the Garlic sauce and bring it to me

6b. Current nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['kitchen is a destination', 'counter is a destination']
  -
    recognized_action: grasp
    slot: ['garlic is an object', 'sauce is an object']
  -
    recognized_action: take
    slot: ['it is an object', 'me is a person']

6c. Desired nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['kitchen counter']
  -
    recognized_action: grasp
    slot: ['garlic sauce']
  -
    recognized_action: bring
    slot: ['here']

------------------------------

7a. GPSR command generator Sentence (cat 1)

move to the sidetable, grasp the Veggie noodles and bring it to me

7b. Current nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['sidetable is a destination']
  -
    recognized_action: grasp
    slot: ['noodles is an object']
  -
    recognized_action: take
    slot: ['it is an object', 'me is a person']

7c. Desired nlu output

sentence_recognition:
  -
    recognized_action: go
    slot: ['sidetable']
  -
    recognized_action: grasp
    slot: ['veggie noodles']
  -
    recognized_action: bring
    slot: ['here']
