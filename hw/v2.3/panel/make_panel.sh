PYTHONPATH=/usr/lib/kicad-nightly/lib/python3/dist-packages/ \
kikit panelize \
    --layout 'hspace: 5mm; cols: 1' \
    --tabs 'spacing: 15mm' \
    --cuts 'type: mousebites; drill: 0.4mm; spacing: 0.8mm; offset: -0.2mm; prolong: 2 mm' \
    --framing 'type: tightframe; width: 3mm' \
    --post 'millradius: 0.8mm' \
    ../main-board/gps-logger-main.kicad_pcb gps-logger-main-panel.kicad_pcb

PYTHONPATH=/usr/lib/kicad-nightly/lib/python3/dist-packages/ \
kikit panelize \
    --layout 'hspace: 5mm; vspace: 5mm; rows: 1; cols: 1' \
    --tabs 'spacing: 20mm' \
    --cuts 'type: mousebites; drill: 0.4mm; spacing: 0.8mm; offset: -0.2mm; prolong: 2 mm' \
    --framing 'type: tightframe; width: 5mm' \
    --post 'millradius: 0.8mm' \
    --debug 'trace: True' \
    ../gnss-board/gps-logger-gnss.kicad_pcb gps-logger-gnss-panel.kicad_pcb

