macroScript ViewportSnap
category: "MODelicus"
tooltip: "Snap viewport view to axis"
(
	temp = viewport.getTM() as EulerAngles

	fn ViewSnap x = 
	(
		global tx=0
		case of
		(
			(-45<=x and x<=45):tx=0
			(45<=x and x<=135):tx=90
			(135<=x and x<=180):tx=180
			(-45>=x and x>=-135):tx=-90
			(-135>=x and x>=-180):tx=-180
		)
		return tx
	)
	temp = eulerAngles (ViewSnap temp.x)(ViewSnap temp.y)(ViewSnap temp.z)
	viewport.setTM (temp as matrix3)
	completeredraw()
)
