# The reasoning VLM — FAU OwlChat HPC endpoint

The `reasoning=vlm` module talks to an **OpenAI-compatible** API hosted by FAU's HPC group
("Owl Chat"). Any OpenAI-compatible endpoint works; nothing in the code is provider-specific beyond
the base URL and model id.

**The API key is not in this repo and must never be committed.** It goes in a `.env` file that is
gitignored, or in your shell environment. If you ever paste it into a file, a log, a commit message
or a chat, treat it as burned and get it rotated.

---

## Configuration

Three environment variables. `OPENAI_BASE_URL` and `OPENAI_API_KEY` are read directly by the client
(`utp/pipeline/reasoning/vlm_gpt5.py`); `UTP_VLM_MODEL` is an optional override.

```bash
# ~/unlocking-the-path/.env      (gitignored — never commit)
OPENAI_BASE_URL=https://chat-llm.hpc.fau.edu/v1
OPENAI_API_KEY=<your Owl Chat virtual key>
UTP_VLM_MODEL=openai/gemma4-vibe
```

Ask the project owner for a key. They are per-user "virtual keys" issued by the Owl Chat service —
do not share one between machines if you can avoid it, because you lose the ability to revoke just
the rover.

The client refuses to start if the key is missing **or still the placeholder**:

```
OPENAI_API_KEY is missing or still the placeholder. Put a valid Owl Chat virtual
key in .env (OPENAI_API_KEY=...) before running reasoning=vlm.
```

That check exists because a placeholder key produced a confusing auth error deep inside a trial
rather than at startup.

## Model

Pinned in `config/methods.yaml`:

```yaml
vlm:
  provider: openai
  model: "openai/gemma4-vibe"   # FAU OwlChat vision id
  model_date: "2026-07-29"      # date this id was verified on the endpoint (vision confirmed)
  temperature: 0.0
  max_tokens: 512
```

**Model ids on this endpoint change without notice.** `azure_ai/gpt-5.5` was removed on 2026-07-29
and everything broke until the id was updated. That is why `model_date` exists and why
`cfg_vlm_model` is written into **every trial record** — a results table whose model id you cannot
reconstruct is not reproducible.

**It must be a vision model.** The reasoner sends an image with every request. A text-only id will
fail or, worse, silently ignore the image and answer from the prompt alone — which looks like a
model that reasons badly rather than one that cannot see.

`temperature: 0.0` is deliberate, for reproducibility. Note the consequence: at temperature 0 the
model repeats an identical answer to an identical question, which is why the reasoner feeds failed
attempts back in (see below).

## How the client uses it

`GPT5Reasoner` in `utp/pipeline/reasoning/vlm_gpt5.py`. One `chat.completions` call per decision:

- **System prompt** = role + the bounded capability/tool list + the decoupling rules.
- **User content** = text (blockage description, actions already completed, actions that failed) plus
  the RGB frame as a base64 `data:` URL.
- **Response** = strict JSON:

```json
{"action_type": "press_button",
 "target_description": "the square blue ADA push plate on the wall left of the door",
 "params": {},
 "rationale": "...",
 "abstain": false}
```

Four rules are enforced in the system prompt, and each exists because of a specific failure:

1. **Never output pixel coordinates or boxes.** A separate detector localizes the description. This
   is the decoupling thesis; violating it deletes the experiment.
2. **Exactly one next action**, not a plan. Multi-step tasks (elevator: call → enter → select →
   exit) are sequenced from the history of completed actions.
3. **Pick the right tool for the control**: wall push plate → `press_button`; badge reader →
   `present_fob`; elevator → `call_elevator` / `select_floor`.
4. **Refusal ≠ abstain.** If no tool can help, or the only opener needs a capability the robot
   lacks, it must emit `action_type="report_unreachable"` with `abstain=true` — the first-class
   refusal verdict. Bare `"none"` is reserved for "cannot see the target yet, keep looking". This
   distinction is contribution C2; conflating them destroys it.

**Failed attempts are fed back.** `observe_failure()` accumulates attempts that did not change the
world, because at temperature 0 an unchanged prompt yields an unchanged answer — measured: three
consecutive `press_button` plans against a *sealed* door, and the whole `M4_unreachable` tier scored
0/20.

**Every attempt is traced before parsing** — prompts sent, response verbatim, error text, latency —
into the trial's artifacts. A parse bug once destroyed its own evidence and left only our paraphrase
for a reviewer to audit.

Malformed JSON or empty content is retried a few times. **A single bad response must never crash a
trial.**

## Verifying connectivity — do this from the test site

The endpoint is a **university HPC service**. It may require the campus network or a VPN, and the
test site is a building with unknown connectivity. **Verify from where the robot will actually run,
not from the lab.** Discovering this on Aug 25 is a lost day.

```bash
set -a; . ~/unlocking-the-path/.env; set +a

# 1. reachable + key valid (expect 200)
curl -s -o /dev/null -w '%{http_code}\n' "$OPENAI_BASE_URL/models" \
     -H "Authorization: Bearer $OPENAI_API_KEY"

# 2. is our pinned model id still listed?
curl -s "$OPENAI_BASE_URL/models" -H "Authorization: Bearer $OPENAI_API_KEY" \
  | python3 -c "import json,sys; print([m['id'] for m in json.load(sys.stdin)['data']])"

# 3. end-to-end, including vision, through our own client
cd ~/unlocking-the-path && . env/.venv/bin/activate
python -c "
from utp.common.config import Config
from utp.pipeline.reasoning.vlm_gpt5 import GPT5Reasoner
r = GPT5Reasoner(Config.load().data['methods']['vlm'])
print('model:', r.model); print('client ok:', r._get_client() is not None)"
```

If step 2 does not list the pinned id, update `model` and `model_date` in `config/methods.yaml`
together, and note it in `EXPERIMENT_LOG.md`. Do not silently swap models between runs — the model
id is part of the result.

**Latency budget.** Each decision is one round trip. `trial_time_budget_s` is 300 and a mission may
need several decisions, so a slow or flaky link eats trials as timeouts, not as reasoning errors.
Record `latency_vlm_s` and watch it; if the site link is bad, that is a finding to report rather
than a nuisance to absorb.

## Failure modes and what they actually mean

| Symptom | Likely cause |
|---|---|
| 401 / 403 | key wrong, expired, or revoked |
| 404 on the model | the id was removed — check `/models`, update `methods.yaml` |
| connection timeout | not on the campus network / no VPN — **most likely at the test site** |
| answers ignore the image | model id is not a vision model |
| identical wrong action repeatedly | `observe_failure` not wired, or history not passed |
| trial fails with empty rationale | JSON parse failure — read the raw trace in artifacts |

## What runs locally instead

Only the **reasoner** needs the network. The grounder (`IDEA-Research/grounding-dino-base`) runs
**on the laptop's GPU** from the local HuggingFace cache. Pre-download it before going to the site:

```bash
HF_HUB_DISABLE_IMPLICIT_TOKEN=1 python -c "
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection as M
for m in ['IDEA-Research/grounding-dino-base','google/owlv2-base-patch16-ensemble']:
    AutoProcessor.from_pretrained(m); M.from_pretrained(m); print('cached', m)"
```

`HF_HUB_DISABLE_IMPLICIT_TOKEN=1` works around a token-pickup issue seen on the workstation.

So: **no network → no reasoning, but grounding still works.** If the site has no usable link, the
fallback that still produces a real result is the `heuristic` reasoner (`ours_no_reasoning`), which
needs no network at all. It is a weaker result, but it is a result — decide that deliberately rather
than discovering it at the door.
