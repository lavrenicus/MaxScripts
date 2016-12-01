macroScript ClipboardReference
	category:"MODelicus"
	toolTip:"Clipboard Reference"
(
	if (getclipboardBitmap()!=undefined) then
	(
		mat = material
		mat = standard showInViewport:true
		mat.diffusemap = bitmaptex()
		t=getclipboardBitmap()
		t.filename = "C:\\temp\\temp.jpg"
		save t
		mat.diffusemap.bitmap = t
		ref = plane length: t.height width: t.width
		ref.material = mat
		ref.name = "reference"
		ref.rotation = eulerAngles -90 0 0
		ref.lengthsegs = 1
		ref.widthsegs = 1
	)
)