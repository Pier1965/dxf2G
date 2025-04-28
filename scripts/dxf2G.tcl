#!/usr/bin/wish -f
# Programma tcl-tk di interfaccia a dxf2G
# Versione 1.0.1
# Data 17/10/2008
#  
wm title . "dxf2G - dxf to G code converter"
wm iconname . "dxf2G"
set w .msgbox
catch {destroy $w}

#set ff [exec man dxf2G]
#puts $ff
#message .helpMsg -aspect 40 -justify left -text $ff
#pack .helpMsg

# menu -------------------------------------------------------------------------------------------------- 
menu .menu -tearoff 0 -type menubar

set m .menu.file  
menu $m -tearoff 0
.menu add cascade -label "File" -menu $m -underline 0 
set modifier Alt 
$m add command -label "Open" -accelerator $modifier+o -command "OpenFile" -underline 0 -command OpenFile
bind . <$modifier-o> "OpenFile"
$m add separator
$m add command -label "Exit" -accelerator $modifier+x -command "exit" -underline 0 
bind . <$modifier-x> "exit"

set h .menu.help
menu $h -tearoff 0
.menu add cascade -label "Help" -menu $h -underline 0
$h add command -label "Help" -accelerator $modifier+h -command "Help" -underline 0 -command Help
bind . <$modifier-h> "Help"


. configure -menu .menu
proc Help {} {
	set h .helpw
	catch {destroy $h}
	toplevel $h
	wm title $h "dxf2G manual page"
	wm iconname $h "dxf2G man"

	frame $h.buttons
	pack $h.buttons -side bottom -fill x -pady 2m
	button $h.buttons.dismiss -text Dismiss -command "destroy $h"
	pack $h.buttons.dismiss  -side left -expand 1

	frame $h.f -highlightthickness 2 -borderwidth 2 -relief sunken
	set t $h.f.text
	text $t -yscrollcommand "$h.scroll set" -setgrid true -width 100 \
	        -height 35 -wrap word -highlightthickness 0 -borderwidth 0
	pack $t -expand  yes -fill both
	scrollbar $h.scroll -command "$t yview"
	pack $h.scroll -side right -fill y
	pack $h.f -expand yes -fill both

	set ff [exec groff -t -e -mandoc -Tascii /usr/local/man/man1/dxf2G.1 | col -bx]
	$t insert end $ff
}
proc OpenFile {} {
	global message
	global file
	set types {
	{{Exchange Drawing Format}	{.dxf}	}
	{{Text Files}				{.txt}	}
	{{All Files}        *             	}
	}
	set file [tk_getOpenFile -filetypes $types]
	if {$file == ""} {
		set message "No file selected..."
		return;
	}
	.filename.nomeFileLbl configure -text "Nome file: $file" 
}	

# fine menu --------------------------------------------------------------------------------------------

set units "m"
set depth 1
set nop 1
set feed 100
set dist 1
set dit 1
set file "none"
set em "G61"

frame .filename -bd 3 -relief ridge -bg yellow 
pack .filename -side top  -padx 5 -pady 5 -fill x

frame .options -borderwidth 5  
pack .options -side top  -padx 5 -pady 5

frame .buttons -bd 2  
pack .buttons -side top  -fill x

checkbutton .options.unitsChkbx -text "Metric Units" -command changeUnits \
		-variable units -onvalue "m" -offvalue "i"
label .options.unitsLbl -text "Millimeters" 

label .options.cuttingDlbl -text "Cutting depth" -justify left
entry .options.cuttingDentry -textvariable depth

label .options.nofpasseslbl -text "Number of passes:" -justify left
entry .options.nofpassesentry -textvariable nop

label .options.feedLbl -text "Feed units/min:"
entry .options.feedEntry -textvariable feed

label .options.distLbl -text "G0 distance from piece:"
entry .options.distEntry -textvariable dist

label .options.ditLbl -text "Cut distance:"
entry .options.ditEntry -textvariable dit

label .options.emLbl -text "Path mode:"
entry .options.emEntry -textvariable em

label .filename.nomeFileLbl -text "File name: $file" -bg yellow

button .buttons.createG -text "Create G-code file" -command createGcode 
button .buttons.exit -text "Exit" -command {exit} 


grid .options.unitsChkbx 	-row 0 -column 0 -sticky "w"
grid .options.unitsLbl 		-row 1 -column 0 -sticky "w"
grid .options.cuttingDentry 	-row 0 -column 2 -sticky "w"
grid .options.cuttingDlbl 	-row 0 -column 1 -sticky "w"
grid .options.nofpasseslbl 	-row 1 -column 1 -sticky "w"
grid .options.nofpassesentry 	-row 1 -column 2 -sticky "w"
grid .options.feedLbl		-row 2 -column 1 -sticky "w"
grid .options.feedEntry 	-row 2 -column 2 -sticky "w"
grid .options.distLbl 		-row 3 -column 1 -sticky "w"
grid .options.distEntry		-row 3 -column 2 -sticky "w"
grid .options.ditLbl		-row 4 -column 1 -sticky "w"
grid .options.ditEntry		-row 4 -column 2 -sticky "w"
grid .options.emLbl		-row 5 -column 1 -sticky "w"
grid .options.emEntry		-row 5 -column 2 -sticky "w"
grid .filename.nomeFileLbl	-row 0 -column 0 -sticky "ew"

pack .buttons.createG		-side left -padx 10 -pady 10
pack .buttons.exit 		-side right -padx 10 -pady 10

proc changeUnits args {
	global units
	global depth
	global nop
	global feed 
	global dist
	global dit
	global file
	global em
	
	if {$units == "m"} {
		.options.unitsChkbx configure -text "Metric Units"
		.options.unitsLbl configure -text "Millimeters"
	} else {
		.options.unitsChkbx configure -text "Imperial Units"
		.options.unitsLbl configure -text "Inch"
	}

	puts "\nUnits= $units"	
	puts "Depth= $depth" 	
	puts "Nop= $nop"
	puts "Feed= $feed"
	puts "Distance= $dist"
	puts "Cutting distance= $dit"
	if {$file != "none"} {
		puts "File name= $file"
	} else {
		puts "No file name provided!" }
	puts "Path mode= $em"
}

proc createGcode {} {
	global units
        global depth
        global nop
        global feed
        global dist
        global dit
        global file
        global em
set ff [exec dxf2G -n $nop -p $depth -d $dist -i $dit -v $feed -u $units -e $em -f $file]
puts $ff
#	exec dxf2G
}
