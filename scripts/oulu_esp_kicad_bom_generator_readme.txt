Put oulu_esp_kicad_bom_generator.py in KICAD_INSTALL_DIR\bin\scripting\plugins

It needs to be there to correctly import some other modules.

Then afterwards in KiCAD, tools -> Generate BOM -> click plus symbol -> add the script from the dir above

Use that script to generate the BOM with correct prices and useless fields removed