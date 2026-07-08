# mission_llm_planner

**The suggester.** This package asks a local AI (Ollama) to turn your English
sentence into a draft plan. Crucially, it **only proposes** — it never drives the
robot, and its output is treated as untrusted until the validator approves it.

## Its role in the pipeline
It's the one place the AI is involved. It sits between your words and the safety
check: words in, a draft plan (JSON) out. Because the AI is boxed in here — its
output must pass the validator before anything moves — the AI is kept **out of the
control loop**, which is a core requirement of the task.

## Input / output
- **Input:** listens on **`/mission/prompt`** (from `mission_interface`).
- **Output:** publishes a draft plan on **`/mission/candidate`**.
- **Who uses the output:** `mission_validator` listens on `/mission/candidate`.
- **External dependency:** the Ollama service (the AI). The address comes from the
  `OLLAMA_HOST` environment variable (in Docker that's `http://ollama:11434`;
  natively it defaults to `http://localhost:11434`). The model name comes from
  `LLM_MODEL` (default `qwen2.5:3b`).

## The file: `mission_llm_planner/llm_planner.py`

- `SYSTEM_PROMPT` — a big instruction block that teaches the AI the rules: which
  routes exist (`perimeter`, `sweep`), which command types are allowed, and
  worked examples (e.g. *"Patrol the perimeter twice"* → `route_name="perimeter",
  loops=2`). Small models follow **concrete examples** far better than abstract
  descriptions, which is why the examples are spelled out.
- `__init__` — reads `LLM_MODEL`, loads the `Mission` JSON schema from
  `mission_schemas`, subscribes to `/mission/prompt`, and creates a publisher on
  `/mission/candidate`.
- `on_prompt()` — runs each time you type something. The important line is the
  call to `ollama.chat(...)` with two key settings:
  - `format=self.schema` — hands the AI the `Mission` shape so its answer is
    **forced** into valid JSON of the right structure (this is "structured
    output").
  - `options={"temperature": 0}` — makes the AI as **deterministic** as possible:
    the same prompt tends to give the same plan, which matters for reproducibility.
  It then publishes the AI's JSON on `/mission/candidate`.
- `_ensure_id()` — a small helper: if the AI forgot to include a `mission_id`, it
  adds a random one, so every plan is uniquely labelled for the audit log.
- `main()` — standard ROS start-up/shut-down wrapper.

## What it deliberately does NOT do
It doesn't check whether the plan is safe or the route exists — that's the
validator's job. Separating "suggest" from "approve" is what makes the AI safe to
use here.


## Deterministic formation-keyword override
The local 3B model is unreliable at one specific choice: mapping an explicitly
stated formation ("line" / "single file" / "wedge") to the right `type`. Because
the schema forces `type` to a valid enum, the model returns a valid-but-wrong
value (e.g. "wedge" for "line"). To make explicit commands 100% reliable,
`_formation_type_from_prompt()` checks the prompt for formation keywords and
`_apply_formation_keyword()` overrides `formation.type` in the candidate JSON
when one is found. The LLM still handles everything else (command type, route,
loops); this only corrects the one categorical field the small model gets wrong.
If you run a larger model that classifies reliably, this override simply never
changes anything.
