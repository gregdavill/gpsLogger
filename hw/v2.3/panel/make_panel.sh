PYTHONPATH=/usr/lib/kicad-nightly/lib/python3/dist-packages/ \
kikit panelize \
    --layout 'hspace: 5mm; cols: 2' \
    --tabs 'spacing: 15mm' \
    --cuts 'type: mousebites; drill: 0.35mm; spacing: 0.5mm; offset: -0.2mm; prolong: 2 mm' \
    --framing 'type: tightframe; width: 3mm' \
    --post 'millradius: 0.8mm' \
    ../main-board/gps-logger-main.kicad_pcb gps-logger-main-panel.kicad_pcb

PYTHONPATH=/usr/lib/kicad-nightly/lib/python3/dist-packages/ \
kikit panelize \
    --layout 'hspace: 5mm; vspace: 2mm; rows: 2; cols: 2' \
    --tabs 'spacing: 20mm' \
    --cuts 'type: mousebites; drill: 0.35mm; spacing: 0.5mm; offset: -0.2mm; prolong: 2 mm' \
    --framing 'type: tightframe; width: 5mm' \
    --post 'millradius: 0.8mm' \
    --debug 'trace: True' \
    ../gnss-board/gps-logger-gnss.kicad_pcb gps-logger-gnss-panel.kicad_pcb

