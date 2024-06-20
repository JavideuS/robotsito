# update_setup_scripts.sh
#!/bin/bash

SETUP_BASH="install/setup.bash"
SETUP_SH="install/setup.sh"

ADDITIONAL_LINES="
# Add venv PYTHONPATH using relative paths
_SCRIPT_DIR=\$(dirname \"\${BASH_SOURCE[0]}\")
VENV_DIR=\"\${_SCRIPT_DIR}/../venv\"
VENV_PYTHONPATH=\$(\"\$VENV_DIR/bin/python3\" -c \"import site; print(site.getsitepackages()[0])\" 2>/dev/null)
export PYTHONPATH=\$VENV_PYTHONPATH:\$PYTHONPATH
"

if ! grep -q "VENV_PYTHONPATH" $SETUP_BASH; then
    echo "$ADDITIONAL_LINES" >> $SETUP_BASH
fi

if ! grep -q "VENV_PYTHONPATH" $SETUP_SH; then
    echo "$ADDITIONAL_LINES" >> $SETUP_SH
fi

