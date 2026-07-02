#!/usr/bin/env bash
#
# dump_pixkit_dwb.bash
# Decodifica in tempo reale i messaggi CAN del PixKit usando il DBC e cantools.
#
# Uso:
#   ./dump_pixkit_dwb.bash [interfaccia_can] [filtri_candump]
#
# Esempi:
#   ./dump_pixkit_dwb.bash                       # can0, filtri di default (530,531,532)
#   ./dump_pixkit_dwb.bash can1                  # can1, filtri di default
#   ./dump_pixkit_dwb.bash can0 530:7FF,531:7FF  # filtri personalizzati
#   ./dump_pixkit_dwb.bash can0 ""               # nessun filtro (tutti i messaggi)

set -euo pipefail

CAN_IF="${1:-can0}"

DBC="/home/miviaware/Desktop/docs_ars/ARS408_id0.dbc"

# --- Controlli ---------------------------------------------------------------
if [[ ! -f "$DBC" ]]; then
    echo "Errore: file DBC non trovato: $DBC" >&2
    exit 1
fi

if ! command -v candump >/dev/null 2>&1; then
    echo "Errore: 'candump' non trovato. Installa can-utils:  sudo apt install can-utils" >&2
    exit 1
fi

if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
    echo "Errore: interfaccia '$CAN_IF' non trovata." >&2
    echo "Suggerimento: configurala con:" >&2
    echo "  sudo ip link set $CAN_IF type can bitrate 500000 && sudo ip link set up $CAN_IF" >&2
    exit 1
fi

# --- Composizione argomento candump -----------------------------------------
echo "Interfaccia: $CAN_IF"
echo "DBC:         $DBC"
echo "Premi Ctrl+C per terminare."
echo "-----------------------------------------------------------------------"

# --- Esecuzione --------------------------------------------------------------
candump "$CAN_IF" | python3 -m cantools decode "$DBC"
