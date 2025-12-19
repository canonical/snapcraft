/**
 * Chart data
*/

// Required
const dataLanes = [
  "core26",
  "core24",
  "core22",
  "core20",
  "core18",
];

// Required
const dataLegend = {
  LTS: {
    description: "Long-Term Support",
    extra: "(10 years)",
    style: "chart__bar--orange",
  },
  ESM: {
    description: "Expanded Security Maintenance",
    extra: "(additional 2 years)",
    style: "chart__bar--aubergine-light",
  }
};

// Required
const dataEntries = [
  {
    startDate: new Date("2018-04-01T00:00:00"),
    endDate: new Date("2028-04-02T00:00:00"),
    taskName: "core18",
    type: "LTS",
  },
  {
    startDate: new Date("2028-04-02T00:00:00"),
    endDate: new Date("2030-04-02T00:00:00"),
    taskName: "core18",
    type: "ESM",
  },
  {
    startDate: new Date("2020-04-01T00:00:00"),
    endDate: new Date("2030-04-02T00:00:00"),
    taskName: "core20",
    type: "LTS",
  },
  {
    startDate: new Date("2030-04-02T00:00:00"),
    endDate: new Date("2032-04-02T00:00:00"),
    taskName: "core20",
    type: "ESM",
  },
  {
    startDate: new Date("2022-04-01T00:00:00"),
    endDate: new Date("2032-05-02T00:00:00"),
    taskName: "core22",
    type: "LTS",
  },
  {
    startDate: new Date("2032-04-02T00:00:00"),
    endDate: new Date("2034-04-01T00:00:00"),
    taskName: "core22",
    type: "ESM",
  },
  {
    startDate: new Date("2024-04-01T00:00:00"),
    endDate: new Date("2034-05-02T00:00:00"),
    taskName: "core24",
    type: "LTS",
  },
  {
    startDate: new Date("2034-05-02T00:00:00"),
    endDate: new Date("2036-04-02T00:00:00"),
    taskName: "core24",
    type: "ESM",
  },
  {
    startDate: new Date("2026-04-01T00:00:00"),
    endDate: new Date("2036-04-01T00:00:00"),
    taskName: "core26",
    type: "LTS",
  },
  {
    startDate: new Date("2036-04-01T00:00:00"),
    endDate: new Date("2038-04-01T00:00:00"),
    taskName: "core26",
    type: "ESM",
  },
];

// Optional
const dataMilestones = [
  {
    name: "Snapcraft 7",
    style: "",
    lanes: {
      start: "core18",
      end: "core22",
    },
  },
  {
    name: "Snapcraft 8",
    style: "",
    lanes: {
      start: "core20",
      end: "core26",
    },
  },
];


/**
 * Component functions
*/

const rowHeight = 40;
const milestoneHeight = 40;
const margin = {
  top: 0,
  right: 40,
  bottom: 20,
};
const kernelVersionWidth = 50;

/**
 * Sort tasks by date.
 * @param {Array} tasks
 * @returns {Array}
 */
function sortTasks(tasks) {
  tasks.sort(function (a, b) {
    return a.endDate - b.endDate;
  });

  tasks.sort(function (a, b) {
    return a.startDate - b.startDate;
  });

  return tasks;
}

/**
 * Add bars to a chart.
 * @param {*} svg - The SVG element in the DOM.
 * @param {Array} tasks
 * @param {Object} legend
 * @param {*} x
 * @param {*} y
 * @param {*} highlightVersion
 * @param {string} chartSelector - The name of the anchor for the target element in the DOM.
 * @param {Number} extraTopMargin - More top margin, for spacing other elements.
 */
function addBarsToChart(
  svg,
  tasks,
  legend,
  x,
  y,
  highlightVersion,
  chartSelector,
  extraTopMargin = 0,
) {
  svg
    .selectAll(".chart")
    .data(tasks, function (d) {
      return d.startDate + d.taskName + d.endDate;
    })
    .enter()
    .append("rect")
    .attr("class", function (d) {
      var className = "";
      const type = d.type

      if (legend[type] === null) {
        return "bar";
      }

      className += " " + legend[type].style;
      return className;
    })
    .attr("y", 0)
    .attr("transform", function (d) {
      return "translate(" + x(d.startDate) + "," + (y(d.taskName) + extraTopMargin) + ")";
    })
    .attr("height", y.bandwidth)
    .attr("width", function (d) {
      return x(d.endDate) - x(d.startDate);
    });
}

/**
 * Add tooltips to bars.
 * @param {*} svg - The SVG element in the DOM.
 */
function addTooltipToBars(svg, legend) {
  const tooltip = createGlobalTooltip();

  svg
    .selectAll(".chart rect")
    .on("mouseover", function (e, d) {
      const type = d.type
      const tooltipStatus = legend[type].description + " " + legend[type].extra
      const dateRange = `${formatDate(d.startDate)} - ${formatDate(d.endDate)}`
      const tooltipContent = tooltipStatus
        ? `${tooltipStatus}: ${dateRange}`
        : dateRange
      tooltip.select(".p-tooltip__message").text(tooltipContent)
      tooltip.classed("u-hide", false)
    })
    .on("mousemove", function (e) {
      tooltip
        .style("left", e.pageX + 2 + "px")
        .style("top", e.pageY - 3 + "px")
    })
    .on("mouseout", function () {
      tooltip.classed("u-hide", true)
    });
}

/**
 * Create a global tooltip element and add it to the DOM.
 */
function createGlobalTooltip() {
  let tooltip = d3.select("body").select(".p-tooltip.is-detached");
  if (tooltip.empty()) {
    tooltip = d3
      .select("body")
      .append("div")
      .attr("class", "p-tooltip is-detached u-hide")
      .attr("style", "pointer-events: none;");
    tooltip
      .append("span")
      .attr("class", "p-tooltip__message")
      .attr("role", "tooltip");
  }

  return tooltip;
}

/**
 * Parse the date to be more readable
 * @param {String} date
 */
function formatDate(date) {
  var formatDate = d3.timeFormat("%b %Y");
  return formatDate(date);
}

/**
 * Add x axis to chart
 * @param {*} svg
 * @param {Int} height
 * @param {*} xAxis
 * @param {Number} extraTopMargin - More top margin, for spacing other elements.
 */
function addXAxis(svg, height, xAxis, extraTopMargin = 0) {
  svg
    .append("g")
    .attr("class", "x axis")
    .attr(
      "transform",
      "translate(0, " + (height - margin.top - margin.bottom + extraTopMargin) + ")",
    )
    .transition()
    .call(xAxis);
}

/**
 * Add y axis to chart
 * @param {*} svg
 * @param {*} yAxis
 * @param {Number} extraTopMargin - More top margin, for spacing other elements.
 */
function addYAxis(svg, yAxis, taskVersions, extraTopMargin = 0) {
  svg
    .append("g")
    .attr("class", "y axis")
    .attr("x", taskVersions ? -margin.left / 1.6 : -margin.left)
    .attr(
      "transform",
      "translate(0, " + extraTopMargin + ")",
    )
    .transition()
    .call(yAxis);
}

/**
 * Add version axis to chart
 * @param {*} svg
 * @param {*} versionAxis
 */
function addVersionAxis(svg, taskVersions) {
  svg.selectAll(".y.axis .tick").each(function (d, i) {
    const existingText = d3.select(this).select("text");
    d3.select(this)
      .append("text")
      .text(taskVersions[i])
      .attr("x", -margin.left)
      .attr("dy", existingText.attr("dy"))
      .attr("fill", "currentColor")
      .attr("class", "chart__kernel-version")
      .attr("text-anchor", "start");
  });
}

/**
 * Clean up unwanted elements on chart put in by d3.js
 * @param {*} svg
 */
function cleanUpChart(svg) {
  svg.selectAll(".domain").remove();
}

/**
 * Add horizontal lines to the x axis.
 * @param {*} svg
 * @param {Int} height
 * @param {Number} extraTopMargin - More top margin, for spacing other elements.
 */
function addXAxisHorizontalLines(svg, height, extraTopMargin = 0) {
  svg.selectAll(".x.axis .tick line")
    .attr("y1", -height - extraTopMargin)
}

/**
 * Add vertical lines to the y axis.
 * @param {*} svg
 * @param {Int} width
 */
function addYAxisVerticalLines(svg, width) {
  svg.selectAll(".y.axis .tick line")
    .attr("x1", width + margin.right)
    .attr("x2", 20)
}

/**
 * Add annotations for milestones.
 * @param {*} chart
 * @param {Array} milestones
 */
function addMilestones(chart, milestones) {

  milestones.forEach((milestone) => {
    var posX = 0
    var posY = 0
    var barHeight = 0

    chart.selectChildren("rect").each(function (d, i) {
      const rect = d3.select(this)
      const transform = rect.attr("transform")
      // Only set the pos vars in the first iteration, so we can skip the rest
      if (milestone.lanes.end === d.taskName && posX === 0) {
        posX = transform.split(",")[0].split("(")[1]
      }
      if (milestone.lanes.start === d.taskName && posY === 0) {
        posY = transform.split(",").slice(-1)[0].replace(")", "")
      }
      barHeight = rect.attr("height")
    })

    let path = d3.path()
    path.moveTo(posX, 24)
    path.lineTo(posX, String(Number(posY) + Number(barHeight)))
    path.closePath()

    const annotation = chart
      .insert("g")

    annotation
      .append("path")
        .attr("d", path)
        .attr("class", "chart__milestone")

    annotation
      .append("text")
        .text(milestone.name)
        .attr("class", "chart-key__label")
        .attr("x", posX)
        .attr("y", 16)
  })
}

/**
 * Build key for supplied chart based on task status.
 * @param {String} chartSelector
 * @param {Object} legend
 */
function buildChartKey(chartSelector, legend) {
  const entries = Object.entries(legend)
  var chartKey = d3
    .select(chartSelector)
    .append("svg")
      .attr("class", "chart-key")
      .attr("width", "550")
      .attr("height", 24 * entries.length);

  entries.forEach((e, i) => {
    var entry = chartKey
      .append("g")
        .attr("class", "chart-key__row")
        .attr("transform", "translate(0, " + 21 * i + ")")
        .attr("height", 24);

    entry
      .append("rect")
        .attr("class", e[1].style)
        .attr("width", 18)
        .attr("height", 14)
        .attr("y", 0);

    entry
      .append("text")
        .text(e[1].description)
        .attr("class", "chart-key__label")
        .attr("x", 24)
        .attr("y", 13);
  });
}

/**
 * Calculate the longest Y-Axis label
 * @param {Array} taskTypes
 */
function calculateLongestLabelWidth(YAxisLabels) {
  var YAxisLabelsCopy = YAxisLabels.slice();
  var longestLabel = YAxisLabelsCopy.sort(function (a, b) {
    return b.length - a.length;
  })[0];
  return longestLabel.length * 7;
}

/**
 * Build chart using supplied element selector and data.
 * @param {String} chartSelector
 * @param {Array} lanes
 * @param {Object} legend
 * @param {Array} entries
 */
function createChart(
  chartSelector,
  lanes,
  legend,
  entries,
  milestones,
  taskVersions,
  removePadding,
  highlightVersion,
) {
  const longestLabelWidth = calculateLongestLabelWidth(lanes);
  if (taskVersions) {
    margin.left = longestLabelWidth + kernelVersionWidth;
  } else {
    margin.left = longestLabelWidth;
  }
  var timeDomainStart;
  var timeDomainEnd;
  const earliestStartDate = d3.min(entries, (d) => d.startDate);
  const latestEndDate = d3.max(entries, (d) => d.endDate);
  if (removePadding) {
    timeDomainStart = earliestStartDate;
    timeDomainEnd = latestEndDate;
  } else {
    timeDomainStart = d3.timeYear.offset(earliestStartDate, -1);
    timeDomainEnd = d3.timeYear.offset(latestEndDate, +1);
  }
  const height = lanes.length * rowHeight;
  const extraHeight = milestones === null ? 0 : milestoneHeight
  var containerWidth = document.querySelector(chartSelector).clientWidth;
  if (containerWidth <= 0) {
    var closestCol = document
      .querySelector(chartSelector)
      .closest('[class*="col-"]');

    if (closestCol.clientWidth <= 0) {
      return;
    }
    containerWidth = closestCol.clientWidth;
  }
  var width = containerWidth - margin.left - margin.right;
  var x = d3
    .scaleTime()
    .domain([timeDomainStart, timeDomainEnd])
    .range([0, width])
    .clamp(true);

  var y = d3
    .scaleBand()
    .domain(lanes)
    .rangeRound([0, height - margin.top - margin.bottom])
    .padding(0.6);

  var xAxis = d3.axisBottom(x);
  var yAxis = d3.axisRight(y).tickPadding(-margin.left).tickSize(0);
  var chartTranslateX = margin.left;

  sortTasks(entries);

  // Build initial chart body
  var svg = d3
    .select(chartSelector)
    .append("svg")
      .attr("class", "chart")
      .attr("width", containerWidth)
      .attr("height", height + margin.top + margin.bottom + extraHeight)
    .append("g")
    .attr("class", "gantt-chart")
    .attr("width", containerWidth - margin.right)
    .attr("height", height)
    .attr(
      "transform",
      "translate(" + chartTranslateX + ", " + margin.top + ")"
    );

  addXAxis(svg, height, xAxis, extraHeight);
  addXAxisHorizontalLines(svg, height, extraHeight);

  addYAxis(svg, yAxis, taskVersions, extraHeight);
  addYAxisVerticalLines(svg, width);

  addBarsToChart(svg, entries, legend, x, y, highlightVersion, chartSelector, extraHeight);
  addTooltipToBars(svg, legend);

  if (milestones !== null) {
    addMilestones(svg, milestones)
  }

  cleanUpChart(svg);
  buildChartKey(chartSelector, legend);
}

/**
 * Main functions
*/

function buildCharts() {
  if (document.querySelector("#support-chart")) {
    delete dataLegend.MAINTENANCE_UPDATES;
    createChart(
      "#support-chart",
      dataLanes,
      dataLegend,
      dataEntries,
      dataMilestones,
    );
  }
}

function clearCharts() {
  const serverDesktopEolOld = document.querySelector("#support-chart");
  if (serverDesktopEolOld) {
    serverDesktopEolOld.innerHTML = "";
  }
}

function debounce(func, wait, immediate) {
  var timeout;

  return function () {
    var context = this,
      args = arguments;
    var later = function () {
      timeout = null;
      if (!immediate) func.apply(context, args);
    };
    var callNow = immediate && !timeout;
    clearTimeout(timeout);
    timeout = setTimeout(later, wait);
    if (callNow) func.apply(context, args);
  };
}

var mediumBreakpoint = 620;

// A bit of a hack, but chart doesn't load with full year axis on first load,
// It has to be loaded once, and then again
// This will need looking into but this fix will work for now
if (window.innerWidth >= mediumBreakpoint) {
  buildCharts();
  setTimeout(function () {
    clearCharts();
    buildCharts();
  }, 0);
}

window.addEventListener(
  "resize",
  debounce(function () {
    if (window.innerWidth >= mediumBreakpoint) {
      clearCharts();
      buildCharts();
    }
  }, 250),
);
