# Writing style guide (from the lab's example theses)

Distilled 2026-10-20 from a full reading of the seven theses in `examples/`, above all Umut Kurt
(Robotics, 2025) and Cem Bilaloğlu (ME, 2022), the author's two favourites; Anıl Gülbeden (EEE,
2025) for describing a robot pipeline with a pen tool; Onur Oydu (Robotics, 2025) for structure.
The detailed reviews, with page-referenced quotes, are in `review_2026-10-20/A1..A3_*.md`.

## 1. Where each kind of content goes

| Content | Chapter |
|---|---|
| Problem statement, notation, assumptions | 3 (start) |
| What a component must deliver, why this family of methods, its mathematics | 3 / 4 |
| The pretrained model by name: one paragraph on how it works, inputs/outputs, what we feed it, its runtime on our hardware, how its confidence is used, how it failed and our fix | 3, at the point of use ("model card", §4) |
| Alternatives in the literature, with one strength and one weakness each | 2 (with a pointer from 3) |
| Threads, nodes, topics, rates, the state machine, parameter values | 3, Implementation Details (Oydu §3.8) |
| Hardware, software versions, test parts, measurement protocol | 5 |
| How well it works, where it breaks, failure cases with remedies | 6 |
| Dependency on the pretrained models, domain gap, what to retrain | 7 |

Kurt's principle: Methodology says **what and why**, Implementation says **how it runs**,
Results says **how well, and where it breaks**. Methodology is almost free of measured numbers.

## 2. Voice and sentences (Kurt)

- "We" for the author's own work in Chapters 3–6 ("we register", "we propose"); impersonal for
  the field ("Seam extraction methods ..."); "this thesis" for scope ("In this thesis, ...");
  "the proposed method / system" as the subject of claims; passive for standard procedures. Never "I".
- Medium sentences (about 20–23 words), one idea each. Connectives that carry the logic:
  *Although, Since, While, In contrast, However, Therefore*.
- Paragraphs of 4–8 sentences that end with a **consequence sentence** ("Therefore, ...",
  "This design choice ensures that ...").
- Define every symbol right after its equation ("where ..."); give every threshold a symbol, a
  value and a reason (Oydu: "k₁ is taken as 0.25 because ...").

## 3. Openers, closers and transitions

- **Chapter:** a roadmap paragraph that names the components in the order they appear; close the
  methodology chapter with a synthesis paragraph ("To conclude, ...").
- **Section, three types:** importance → goal ("X is a crucial step in Y. Given ..., the goal is
  to ..."); field framing; chained next challenge ("Once the parts are registered, the next
  challenge is ...").
- **Transitions:** "Once X ..., the next challenge is Y"; "Building on this, ..."; backward and
  forward references by section number.

## 4. Pretrained and third-party models: the model card

Kurt never opens the network; Gülbeden gives only role, data and runtime. For FoundationPose and
SAM2 that looks thin, so combine them with Oydu's architecture paragraph. Per model:

1. **Task first** (no brand name): what the stage must deliver in the cell, as a requirement.
2. **Families of solutions**, one pro and one con each, and why the chosen family fits
   (CAD-based, zero-shot, promptable). Details of the alternatives go to Chapter 2.
3. **One architecture paragraph** with the paper's figure, attributed ("adapted from [n]").
4. **Training data and whether we fine-tuned** (we did not).
5. **Inputs and outputs** with types and frames (Bilaloğlu's "The output of ... are ...").
6. **How we use it:** prompts, masks, iterations, where it runs (machine, container, rate).
7. **How its confidence is used downstream**, and **the failure we saw and our fix**.
8. **Equations only where our own maths consumes its output** (pose composition, ICP, fusion).

State once, Bilaloğlu style, what is and is not explained about tools we did not write.

## 5. Our own algorithms, step by step

Kurt's chain: context → goal ("Given ..., the goal is ...") → the standard approach and where it
fails here → "In this thesis, we ..." → decomposition into sub-tasks (one subsection each) →
notation → assumption → equations → what the output means → one summary figure with a
step-numbered caption → property analysis (invariance, sensitivity, cost) → forward reference
to the experiment that justifies it.

Unlike the examples, **use typeset algorithm boxes** (`algorithm` + `algorithmic`) for every
non-trivial loop (multi-view refinement, curved seams, the roll DP, the transit ladder, the
descent, the two extractors), each followed by its line-by-line explanation. The committee reads
these easily, and none of the example theses has enough of them.

## 6. Justifying a design decision

- Kurt: requirement "in our application" → options and their weaknesses → choice → consequence →
  a later experiment that justifies it → failure and remedy.
- Gülbeden (calibration, pen contact): **observed symptom → (honest) wrong first hypothesis →
  literature → model → algorithm → procedure → error against distance**. Use it for the
  calibration chain, the cable wrap, the wrist buzz, the plate straddle.
- Bilaloğlu: specification → closed-form calculation → caveat ("theoretical maximum") →
  measurement → reconciliation in the discussion.
- Trade-offs with a visible consequence ("a lower touch force spares the pen but misses tilted
  plates").
- Scope fences ("Actual welding is outside the scope of this thesis; ...").
- Relate an error to the task tolerance ("2.5 mm against a 4 mm wide tack mark").

## 7. Figures, tables, equations

- Introduce each figure in a sentence before it, with a small verb set: *illustrates, is shown in,
  is summarised in, compares*. Short noun-phrase captions; a step-numbered caption for a pipeline
  figure; the reading guide (colours, panels, axes) in the text.
- Reproduced figures: "adapted from [n]".
- One parameter table (Bilaloğlu has several; Kurt has none).
- A hand-drawn node graph, not an rqt_graph screenshot (Oydu's is unreadable in print).

## 8. Implementation details (Oydu §3.8 + Gülbeden §3.8)

1. Architecture rationale: why these process boundaries, where Docker and the GPUs sit, why a
   WebSocket between the container and ROS 2.
2. Node inventory: *functional name (executable)*, role, subscribes/publishes, services/actions,
   thresholds with equation links.
3. A rates table (node → Hz, with the reason for each rate).
4. A state machine of the cell with labelled transitions **and failure/abort edges** (neither
   example draws failure edges).
5. Software versions in Chapter 5.

## 9. Limitations (Kurt)

At the point of evidence, in a fixed sequence: observation → mechanism → fix → residual risk →
principled future remedy; then a scoped limitation with its condition ("A current limitation of
the proposed method is that ... when ..."). A closing reassurance where it is true.

## 10. Do not copy these mistakes (all passed a committee)

- Tools used but not cited (Kurt: LightGlue, RANSAC, Levenberg–Marquardt, A*). Cite every tool.
- A key component never written down (Kurt's information matrix). Write every formula used.
- Claims with no experiment behind them; abstract claims from simulation stated as hardware
  (Akbaba). Claim only what Chapter 6 shows.
- Text and figure disagreeing (Bilaloğlu), heading typos ("FEATURE WORK"), empty equation
  numbers, numeric results numbered as equations.
- Maths errors in restated textbook material (Oydu: the essential matrix "has 7 DoF").
- Single runs presented as performance (most of them). Report n, mean, std, median, max.

## 11. Phrase bank (Kurt unless marked)

| Function | Form |
|---|---|
| Role | "This estimation serves as the backbone for ..." |
| Choice | "In this thesis work, X coupled with Y is used to ..." |
| Requirement | "In our application, ... is especially important." |
| Interpretation | "This step can be interpreted as the projection of ... onto ..." |
| Practice | "In practice, we observed that ..." |
| Evidence → adoption | "For this reason, we adopt ... in our implementation." |
| Intuition | "Conceptually, this can be imagined as ..." |
| Trade-off | "While this increases ..., it also leads to ..." |
| Residual error | "The main reason for the remaining ... is ..." |
| Reassurance | "Nevertheless, ... remains suitable for ..." |
| Dependency | "The accuracy of the proposed system depends strongly on its perception modules: ..." |
| Dataset motivation | "This scarcity motivates the generation of datasets tailored for ..." |
| Novelty hedge (Bilaloğlu) | "To the best of our knowledge, ..." |
| Scope policy (Bilaloğlu) | "Rather than describing how these tools work, this section explains how they were used to ..." |
