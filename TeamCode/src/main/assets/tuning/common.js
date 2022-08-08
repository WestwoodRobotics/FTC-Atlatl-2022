// TODO: time-interpolate data

function fitLinearWithScaling(xs, ys) {
  const xNorm = xs.reduce((acc, x) => Math.max(acc, Math.abs(x)), 0);
  const yNorm = ys.reduce((acc, x) => Math.max(acc, Math.abs(x)), 0);
  const data = xs.map((x, i) => [x / xNorm, ys[i] / yNorm]);
  const result = regression.linear(data);
  const [m, b] = result.equation;
  return [m * yNorm / xNorm, b * yNorm];
}

// no output for first pair
function numDerivOnline(xs, ys) {
  if (xs.length !== ys.length) {
    throw new Error(`${xs.length} !== ${ys.length}`);
  }

  return ys
    .slice(1)
    .map((y, i) => (y - ys[i]) / (xs[i + 1] - xs[i]));
}

// no output for first or last pair
function numDerivOffline(xs, ys) {
  return ys
    .slice(2)
    .map((y, i) => (y - ys[i]) / (xs[i + 2] - xs[i]));
}

const CPS_STEP = 0x10000;

function inverseOverflow(input, estimate) {
  if (!input && input !== 0) {
    return input;
  }

  if (input % 4 !== 0) {
    throw new Error(`${input} % 4 !== 0`);
  }

  // get close with numerical estimate
  while (Math.abs(estimate - input) > CPS_STEP / 2.0) {
    if (input < estimate) {
      input += CPS_STEP;
    } else {
      input -= CPS_STEP;
    }
  }

  // snap to nearest multiple of 20
  while (input % 20 !== 0) {
    if (input % 20 > 10) {
      input -= CPS_STEP;
    } else {
      input += CPS_STEP;
    }
  }

  return input;
}

// no output for first or last pair
function fixVels(ts, xs, vs) {
  return numDerivOffline(ts, xs).map((est, i) => inverseOverflow(vs[i + 1], est));
}

// data comes in pairs
function newLinearRegressionChart(container, xs, ys, options, onChange) {
  if (xs.length !== ys.length) {
    throw new Error(`${xs.length} !== ${ys.length}`);
  }

  // cribbed from https://plotly.com/javascript/plotlyjs-events/#select-event
  const color = '#777';
  const colorLight = '#bbb';

  let mask = xs.map(() => true);

  const [m, b] = fitLinearWithScaling(xs, ys);

  if (onChange) onChange(m, b);

  const minX = xs.reduce((a, b) => Math.min(a, b), 0);
  const maxX = xs.reduce((a, b) => Math.max(a, b), 0);

  const chartDiv = document.createElement('div');
  Plotly.newPlot(chartDiv, [{
    type: 'scatter',
    mode: 'markers',
    x: xs,
    y: ys,
    name: 'Samples',
    // markers seem to respond to selection 
    marker: {color: mask.map(b => b ? color : colorLight), size: 5},
  }, {
    type: 'scatter',
    mode: 'lines',
    x: [minX, maxX],
    y: [m * minX + b, m * maxX + b],
    name: 'Regression Line',
    line: {color: 'red'}
  }], {
    title: options.title || '',
    // sets the starting tool from the modebar
    dragmode: 'select',
    showlegend: false,
    hovermode: false,
    width: 600,
  }, {
    // 'select2d' left
    modeBarButtonsToRemove: ['zoom2d', 'pan2d', 'lasso2d', 'zoomIn2d', 'zoomOut2d', 'autoScale2d', 'resetScale2d'],
  });

  const results = document.createElement('p');

  function setResultText(m, b) {
    results.innerText = `${options.slope || 'slope'}: ${m}, ${options.intercept || 'y-intercept'}: ${b}`;
  }
  setResultText(m, b);

  function updatePlot() {
    Plotly.restyle(chartDiv, 'marker.color', [
      mask.map(b => b ? color : colorLight)
    ], [0]);

    const [m, b] = fitLinearWithScaling(
      xs.filter((_, i) => mask[i]),
      ys.filter((_, i) => mask[i]),
    );
    setResultText(m, b);
    if (onChange) onChange(m, b);

    const minX = xs.reduce((a, b) => Math.min(a, b));
    const maxX = xs.reduce((a, b) => Math.max(a, b));

    Plotly.restyle(chartDiv, {
      x: [[minX, maxX]],
      y: [[m * minX + b, m * maxX + b]],
    }, [1]);
  }

  let pendingSelection = null;

  chartDiv.on('plotly_selected', function(eventData) {
    pendingSelection = eventData;
  });

  function applyPendingSelection(b) {
    if (pendingSelection === null) return false;

    for (const pt of pendingSelection.points) {
      mask[pt.pointIndex] = b;
    }

    Plotly.restyle(chartDiv, 'selectedpoints', [null], [0]);

    pendingSelection = null;

    return true;
  }

  const includeButton = document.createElement('button');
  includeButton.innerText = '[i]nclude';
  includeButton.addEventListener('click', () => {
    if (!applyPendingSelection(true)) return;
    updatePlot();
  });

  const excludeButton = document.createElement('button');
  excludeButton.innerText = '[e]xclude';
  excludeButton.addEventListener('click', () => {
    if (!applyPendingSelection(false)) return;
    updatePlot();
  });

  document.addEventListener('keydown', e => {
    if (e.key === 'i') {
      if (!applyPendingSelection(true)) return;
      updatePlot();
    } else if (e.key === 'e') {
      if (!applyPendingSelection(false)) return;
      updatePlot();
    }
  });

  while (container.firstChild) {
    container.removeChild(container.firstChild);
  }

  const buttons = document.createElement('div');
  buttons.appendChild(includeButton);
  buttons.appendChild(excludeButton);

  const bar = document.createElement('div');
  bar.setAttribute('class', 'bar');
  bar.appendChild(buttons);

  bar.appendChild(results);

  container.appendChild(bar);
  container.appendChild(chartDiv);

  return function(xsNew, ysNew) {
    if (xsNew.length !== ysNew.length) {
      throw new Error(`${xsNew.length} !== ${ysNew.length}`);
    }

    xs = xsNew;
    ys = ysNew;
    mask = xs.map(() => true);

    Plotly.restyle(chartDiv, {
      x: [xs],
      y: [ys],
    }, [0]);

    updatePlot();
  };
}
