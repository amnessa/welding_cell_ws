# Figures

One file per figure, named by its LaTeX label without the prefix, e.g. `intro_cell.pdf`
for `fig:intro_cell`. Prefer PDF (vector) for plots and diagrams, PNG/JPG for photos.

When a figure is ready, replace its `\figplaceholder{fig:x}{...}{caption}` with:

```latex
\begin{figure}[htbp]\centering
\includegraphics[width=0.9\textwidth]{x}
\caption{...}\label{fig:x}
\end{figure}
```

The list of planned figures, with their sources, is in `../THESIS_PLAN.md`.
