# Hermes integration

Install the canonical Agent Bounties skill without duplicating its contents:

```sh
hermes skills install https://raw.githubusercontent.com/NSPG13/agent-bounties/main/skills/agent-bounties/SKILL.md
```

Start a fresh Hermes session with `/reset` after installation so the new skill
is loaded. The skill queries canonical claimable inventory, distinguishes
unfunded and stale results, and emits one deterministic next action.

Run `python scripts/check-hermes-integration.py` to validate the integration
and its offline fixtures.
