(define (problem minimum_required_facts)
	(:domain gpsr) ; General Purpose Service Robot (in a home environment)
	(:objects

		; locations
		 l--office l--sidetable l--bookshelf l--bedside l--bedroom l--kitchen_table l--dinning_table l--dinning_room l--fridge l--desk - location
		 l--bar l--sofa l--kitchen_chair l--TV_stand l--cupboard l--toilet l--closet l--living_room l--sink l--kitchen - location
		 l--bookcase l--living_table l--sideshelf l--table l--bathroom l--start l--kitchen_cabinet l--door l--bedroom_chair l--cabinet - location
		 l--wardrobe l--bed l--nightstand l--counter l--center_table l--drawer l--couch l--corridor l--hallway l--coffee_table - location
		
		; people
		 p--summer p--madison p--lewis p--charles p--toby p--thomas p--logan p--harrison p--liam p--emily - person
		 p--blake p--noah p--josh p--aaron p--edward p--ethan p--scarlett p--theo p--luke p--samuel - person
		 p--amy p--jack p--jacob p--alice p--eleanor p--arthur p--james p--barbara p--lucy p--ryan - person
		 p--william p--harvey p--mason p--emma p--daniel p--max p--ken p--rosie p--alexander p--harry - person
		 p--alex p--brooke p--faith p--jackie p--sophia p--noperson p--rose p--ava p--emilia p--joshua - person
		 p--connor p--sienna p--florence p--katie p--sarah p--oliver p--chloe p--tyler p--jane p--john - person
		 p--oscar p--isaac p--amelia p--amelie p--nathan p--isabelle p--hanna p--tommy p--will p--henry - person
		 p--sophie p--amber p--charlotte p--grace p--erika p--erik p--ivy p--peter p--zoe p--mia - person
		 p--isabella p--abigail p--matthew p--brian p--martha p--lily p--jamie p--michael p--samantha p--ella - person
		 p--daisy p--freddie p--evan p--louis p--dylan p--skyler p--charlie p--seth p--person p--adam - person
		 p--olivia p--elliot p--paige p--joseph - person

		; objects
		 obj--bananas obj--laptop obj--peaches obj--milk obj--blueberries obj--pens obj--ham obj--tablet obj--peanut obj--sugar - object
		 obj--orange obj--peanuts obj--manju obj--plate obj--coffee obj--chewing_gums obj--blackberries obj--food obj--oranges obj--dryer - object
		 obj--watermelon obj--towel obj--chips obj--chocolate_tablet obj--bread obj--tray obj--onion obj--cider obj--candy obj--mints - object
		 obj--glasses obj--newspaper obj--cookies obj--knifes obj--red_bull obj--tea obj--almond obj--bowls obj--sponge obj--burger - object
		 obj--almonds obj--noodles obj--container obj--pen obj--yogurt obj--forks obj--charger obj--choth obj--knife obj--blackberry - object
		 obj--pepper obj--banana obj--rum obj--whiteboard_cleaner obj--lotion obj--toothbrush obj--snack obj--cookie obj--grapes obj--comb - object
		 obj--toilet_paper obj--crackers obj--sushi obj--biscuits obj--biscuit obj--cup obj--magazine obj--drink obj--cheese obj--apple - object
		 obj--can obj--kleenex obj--magazines obj--trays obj--sake obj--donuts obj--beer obj--chocolate_egg obj--toothpaste obj--containers - object
		 obj--fork obj--pencil obj--paprika obj--sprite obj--chocolate_bar obj--coke obj--strawberries obj--lemons obj--hot_chocolate obj--iced_tea - object
		 obj--whisky obj--rice obj--shampoo obj--strawberry obj--books obj--lemonade obj--apples obj--cake obj--newspapers obj--cans - object
		 obj--vodka obj--snacks obj--candies obj--senbei obj--pie obj--cloth obj--blueberry obj--bowl obj--onions obj--cream - object
		 obj--lemon obj--peach obj--water obj--book obj--cereals_bar obj--plates obj--soap obj--pringles obj--pizza obj--pear - object
		 obj--notebook obj--chewing_gum obj--bottles obj--cleaning_stuff obj--glass obj--toiletries obj--juice obj--bottle obj--wine obj--salt - object
		
)
	(:init
		; the robot at start is in the entrance of the house
		(at_r l--start)
		; the robot gripper is empty at the start
		(gripper_empty)
	)
	(:goal )
)
