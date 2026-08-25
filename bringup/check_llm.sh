#!/usr/bin/env bash
# Settle the VLM endpoint empirically: which base URL works, is the pinned model still listed,
# and does vision actually work end to end.
#
#     bash bringup/check_llm.sh
#
# Run this FROM THE TEST SITE, not from the lab. docs/LLM_ENDPOINT.md is blunt about why:
# the endpoint is a university HPC service that may need the campus network or a VPN, and
# "discovering this on Aug 25 is a lost day".
#
# WHY IT TRIES TWO HOSTS: the repo's own docs disagree.
#   docs/owlchat_llm_guide.md (2026-07-02) says https://chat.hpc.fau.edu/api/v1 and warns that
#     chat-llm.hpc.fau.edu is the raw LiteLLM backend which rejects keys with token_not_found_in_db.
#   docs/LLM_ENDPOINT.md and HARDWARE_SPECS.md both say https://chat-llm.hpc.fau.edu/v1.
# Rather than pick a side, ask the server.
#
# Prints no key material. Ever.
set -uo pipefail

ENVF="${UTP_PIPELINE_ENV:-$HOME/unlocking-the-path/.env}"
[ -f "$ENVF" ] || { echo "no $ENVF -- see docs/LLM_ENDPOINT.md" >&2; exit 1; }
set -a; . "$ENVF"; set +a

if [ -z "${OPENAI_API_KEY:-}" ] || case "$OPENAI_API_KEY" in sk-REPLACE*) true;; *) false;; esac; then
    cat >&2 <<EOF
OPENAI_API_KEY is still the placeholder in $ENVF.

  1. log in at https://chat.hpc.fau.edu/  (FAU SSO)
  2. profile icon -> Account -> API Keys -> Show -> Create API key
  3. put the sk-... token in $ENVF as OPENAI_API_KEY=...

Nothing else needs changing. Re-run this script afterwards.
EOF
    exit 1
fi
echo "key: loaded (${#OPENAI_API_KEY} chars, not printed)"

seen=""
CANDIDATES="${OPENAI_BASE_URL:-} https://chat.hpc.fau.edu/api/v1 https://chat-llm.hpc.fau.edu/v1"
WORKING=""
echo
echo "== 1/3  which base URL accepts this key? =="
for url in $CANDIDATES; do
    [ -n "$url" ] || continue
    case " $seen " in *" $url "*) continue;; esac; seen="${seen:-} $url"
    code=$(curl -s -o /tmp/_llm_body -w '%{http_code}' -m 20 "$url/models" \
           -H "Authorization: Bearer $OPENAI_API_KEY" 2>/dev/null)
    if [ "$code" = "200" ]; then
        echo "   OK    $url  (HTTP 200)"
        [ -z "$WORKING" ] && WORKING="$url"
    else
        # surface the server's own reason -- token_not_found_in_db is the documented wrong-host tell
        why=$(head -c 200 /tmp/_llm_body 2>/dev/null | tr -d '\n')
        echo "   --    $url  (HTTP $code) ${why:+: $why}"
    fi
done
rm -f /tmp/_llm_body
[ -n "$WORKING" ] || { echo; echo "NO base URL accepted the key."; echo \
  "  * on the campus network / VPN?  * key valid and not revoked?"; exit 1; }
echo "   -> use OPENAI_BASE_URL=$WORKING"
if [ "$WORKING" != "${OPENAI_BASE_URL:-}" ]; then
    echo "   !! this differs from .env -- update OPENAI_BASE_URL and note it in EXPERIMENT_LOG.md"
fi

echo
echo "== 2/3  is the pinned model still listed? =="
echo "   pinned: ${UTP_VLM_MODEL:-<unset>}"
curl -s -m 25 "$WORKING/models" -H "Authorization: Bearer $OPENAI_API_KEY" \
 | python3 -c '
import json,sys,os
try: ids=[m["id"] for m in json.load(sys.stdin).get("data",[])]
except Exception as e: print("   could not parse /models:",e); raise SystemExit(0)
want=os.environ.get("UTP_VLM_MODEL","")
print(f"   {len(ids)} models listed")
if want in ids: print(f"   OK    {want} is present")
else:
    print(f"   !!    {want} is NOT listed -- ids change without notice (methods.yaml records")
    print(f"         azure_ai/gpt-5.5 being removed 2026-07-29). Update model AND model_date")
    print(f"         together, and log it: the model id is part of the result.")
    vis=[i for i in ids if any(k in i.lower() for k in ("vl","vision","gemma","gpt","claude","gemini"))]
    print("   candidates:", ", ".join(vis[:12]) or "(none obvious)")'

echo
echo "== 3/3  end-to-end, including VISION, through our own client =="
cd "$HOME/unlocking-the-path" || exit 1
OPENAI_BASE_URL="$WORKING" env/.venv/bin/python validation/vlm_smoke.py 2>&1 | tail -20
