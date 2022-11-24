ROM2C=~/Developer/strangecart/rom2c_$CPUTYPE
$ROM2C -a rom994a assets/994arom.bin 994arom.c
$ROM2C -a grom994a assets/994agrom.bin 994agrom.c

$ROM2C -a romparsec assets/PARSECC.BIN parsecrom.c
$ROM2C -a gromparsec assets/PARSECG.BIN parsecgrom.c

$ROM2C -a rominvaders assets/TI-InvaC.BIN invadersrom.c
$ROM2C -a grominvaders assets/TI-InvaG.BIN invadersgrom.c
 
# Multicolor demo program.
$ROM2C -a multicolor assets/multicolor.bin multicolorrom.c

