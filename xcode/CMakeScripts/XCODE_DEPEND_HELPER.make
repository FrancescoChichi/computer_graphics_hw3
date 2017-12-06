# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.pathtrace.Debug:
/Users/s1r/Desktop/computer_graphics_hw3/bin/Debug/pathtrace:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw3/bin/Debug/pathtrace


PostBuild.pathtrace.Release:
/Users/s1r/Desktop/computer_graphics_hw3/bin/Release/pathtrace:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw3/bin/Release/pathtrace


PostBuild.pathtrace.MinSizeRel:
/Users/s1r/Desktop/computer_graphics_hw3/bin/MinSizeRel/pathtrace:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw3/bin/MinSizeRel/pathtrace


PostBuild.pathtrace.RelWithDebInfo:
/Users/s1r/Desktop/computer_graphics_hw3/bin/RelWithDebInfo/pathtrace:
	/bin/rm -f /Users/s1r/Desktop/computer_graphics_hw3/bin/RelWithDebInfo/pathtrace




# For each target create a dummy ruleso the target does not have to exist
