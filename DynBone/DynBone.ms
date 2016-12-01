macroScript DynBoneHelper
	category:"MODelicus"
	toolTip:"Dynamic Bone Helper"
(
	rollout DynBone "Dynamic Bone Helper" width:100 height:100
	(

		button 'DynBone' "Make D" pos:[10,10] width:80 height:80 toolTip:"Make Dynamic Bone" align:#left
		fn drawLine pointA pointB =
		(
		  ss = SplineShape pos:pointA
		  addNewSpline ss
		  addKnot ss 1 #corner #line PointA
		  addKnot ss 1 #corner #line PointB
		  updateShape ss
		  ss
		)

		on DynBone pressed  do
		(
			if($ != undefined) then
			(
				if $ == $selection then
				(
					ObjA = $[1]
					ObjB = $[2]
					if(ObjA.position!=undefined and ObjB.position!=undefined) then
					(
						Dline = drawLine ObjA.position ObjB.position
						Dline.name = ObjB.name + "_Line"
						Dline.parent = ObjA
						addmodifier Dline (flex())
						Dline.modifiers[#Flex].strength = 0.5
						Dline.modifiers[#Flex].sway = 10
						ObjB.pos.controller = Path_constraint()
						ObjB.pos.controller.path = Dline
						ObjB.pos.controller.percent = 100
						ObjB.pos.controller.follow = on
						deleteKeys ObjB.pos.controller #allKeys
						
					)
				)
				else
				(
					messagebox("Select two object!")
				)
			)
			else messagebox("Select two object, first BASE object, second dynamic object")
		)
	)
	createDialog DynBone width:100 height:100
)