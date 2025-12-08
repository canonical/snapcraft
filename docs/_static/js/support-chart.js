// Chart data

// Defines the horizontal swimlanes. Required.
const dataLanes = [
  "core24",
  "core22",
  "core20",
  "core18",
]

// Defines the chart legend. Required.
const dataLegend = {
  LTS: {
    description: "LTS standard security maintenance",
    extra: "(5 years)",
    style: "chart__bar--orange",
  },
  ESM: {
    description: "LTS Expanded Security Maintenance",
    extra: "(5 years)",
    style: "chart__bar--aubergine-light",
  },
  Pro: {
    description: "Legacy add-on for Ubuntu Pro",
    extra: "(5 years)",
    style: "chart__bar--black",
  },
  release: {
    description: "Initial release",
    style: "chart__milestone-release",
    width: 3,
    height: 14,
  },
};

// Defines elements inside the swimlanes. Required.
const dataEntries = [
  {
    startDate: new Date(2018, 4, 1),
    endDate: new Date(2023, 3, 31),
    taskName: "core18",
    type: "LTS",
  },
  {
    startDate: new Date(2023, 4, 1),
    endDate: new Date(2028, 3, 31),
    taskName: "core18",
    type: "ESM",
  },
  {
    startDate: new Date(2028, 4, 1),
    endDate: new Date(2033, 3, 31),
    taskName: "core18",
    type: "Pro",
  },
  {
    startDate: new Date(2020, 4, 1),
    endDate: new Date(2025, 3, 31),
    taskName: "core20",
    type: "LTS",
  },
  {
    startDate: new Date(2025, 4, 1),
    endDate: new Date(2030, 3, 31),
    taskName: "core20",
    type: "ESM",
  },
  {
    startDate: new Date(2030, 4, 1),
    endDate: new Date(2035, 3, 31),
    taskName: "core20",
    type: "Pro",
  },
  {
    startDate: new Date(2022, 4, 1),
    endDate: new Date(2027, 3, 31),
    taskName: "core22",
    type: "LTS",
  },
  {
    startDate: new Date(2027, 4, 1),
    endDate: new Date(2032, 3, 31),
    taskName: "core22",
    type: "ESM",
  },
  {
    startDate: new Date(2032, 4, 1),
    endDate: new Date(2037, 3, 31),
    taskName: "core22",
    type: "Pro",
  },
  {
    startDate: new Date(2024, 4, 1),
    endDate: new Date(2029, 3, 31),
    taskName: "core24",
    type: "LTS",
  },
  {
    startDate: new Date(2029, 4, 1),
    endDate: new Date(2034, 3, 31),
    taskName: "core24",
    type: "ESM",
  },
  {
    startDate: new Date(2034, 4, 1),
    endDate: new Date(2039, 3, 31),
    taskName: "core24",
    type: "Pro",
  },
]

// Defines milestones. Optional.
const dataMilestones = [
  {
    name: "Snapcraft 7",
    id: "snapcraft-7",
    lanes: {
      released: "core20",
      lowest: "core18",
      highest: "core22",
    },
  },
  {
    name: "Snapcraft 8",
    id: "snapcraft-8",
    lanes: {
      released: "core24",
      lowest: "core20",
      highest: "core24",
    },
  },
]


// Component functions

const rowHeight = 40;
const milestoneHeight = 20;
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
 * @param {string} chartContainer - The name of the anchor for the target element in the DOM.
 * @param {Number} extraTopMargin - More top margin, for spacing other elements.
 */
function addBarsToChart(
  svg,
  tasks,
  legend,
  x,
  y,
  highlightVersion,
  chartContainer,
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
function addMilestones(chart, milestones, height) {

  milestones.forEach((milestone) => {
    let posX

    chart.selectChildren("rect").each(function (d, i) {
      const rect = d3.select(this)
      const transform = rect.attr("transform")
      // Only set the pos vars in the first iteration, so we can skip the rest
      if (d.taskName === milestone.lanes.released && posX === undefined) {
        posX = Math.round(Number(transform.split(",")[0].split("(")[1]))
        releaseY = Math.round(Number(transform.split(",").slice(-1)[0].replace(")", "")))
      }
    })

    // Add the line
    let path = d3.path()
    path.moveTo(posX, milestoneHeight)
    path.lineTo(posX, height)
    path.closePath()

    const annotation = chart
    .insert("g")

    annotation
      .append("path")
        .attr("d", path)
        .attr("stroke-width", "2.5px")
        .attr("data-version", milestone.id)
        .attr("class", "chart__milestone")
        .attr("shape-rendering", "crispEdges")
        .attr("vector-effect", "non-scaling-stroke")

    // Add the text
    annotation
      .append("text")
        .text(milestone.name)
        .attr("data-version", milestone.id)
        .attr("class", "chart-key__label")
        .attr("x", posX)
        .attr("y", 16)
  })
}

/**
 * Build key for supplied chart based on task status.
 * @param {String} chartContainer
 * @param {Object} legend
 */
function buildChartKey(chartContainer, legend) {
  const entries = Object.entries(legend)
  var chartKey = d3
    .select(chartContainer)
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
        .attr("width", e[1].width ? e[1].width : 18 )
        .attr("height", e[1].height ? e[1].height : 14)
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
 * Build menu for milestones.
 * @param {String} chartContainer - The support chart div.
 * @param {Object} milestones - The milestone data.
 */
function addMilestoneMenu(chartContainer, milestones) {
  const container = document.querySelector(chartContainer)
  const menu = document.createElement("div")
  menu.id = "support-chart-menu"
  container.insertBefore(menu, container.firstChild)

  // Create div.support-chart-menu-btn
  const div = menu.appendChild(document.createElement("div"))
  div.classList.add("support-chart-menu-btn")

  // Create button
  const btn = div.appendChild(document.createElement("button"))
  btn.ariaControls = "support-chart-list"
  btn.ariaExpanded = "false"
  btn.textContent = "All versions"

  // Create ul#support-chart-list
  const ul = div.appendChild(document.createElement("ul"))
  ul.id = "support-chart-list"
  ul.classList.add("support-chart-hidden")
  ul.ariaHidden = "true"

  // Create default item
  let li = ul.appendChild(document.createElement("li"))
  li.textContent = "All versions"
  li.ariaSelected = "true"
  li.dataset.version = "all"

  // Create items for milestones
  for (let milestone of milestones.reverse()) {
    let li = ul.appendChild(document.createElement("li"))
    li.textContent = milestone.name
    li.ariaSelected = "false"
    li.dataset.version = milestone.id
  }
  ul.childNodes.forEach(e => e.addEventListener("click", (e) => chartMenuSelect(e.target)))
  btn.addEventListener("click", () => chartButtonToggle())
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
 * @param {String} chartContainer
 * @param {Array} lanes
 * @param {Object} legend
 * @param {Array} entries
 */
function createChart(
  chartContainer,
  lanes,
  legend,
  entries,
  milestones,
  taskVersions,
  removePadding,
  highlightVersion,
) {
  const longestLabelWidth = calculateLongestLabelWidth(lanes)
  if (taskVersions) {
    margin.left = longestLabelWidth + kernelVersionWidth
  } else {
    margin.left = longestLabelWidth
  }
  var timeDomainStart
  var timeDomainEnd
  const earliestStartDate = d3.min(entries, (d) => d.startDate)
  const latestEndDate = d3.max(entries, (d) => d.endDate)
  if (removePadding) {
    timeDomainStart = earliestStartDate
    timeDomainEnd = latestEndDate
  } else {
    timeDomainStart = d3.timeYear.offset(earliestStartDate, -1)
    timeDomainEnd = d3.timeYear.offset(latestEndDate, +1)
  }
  const height = lanes.length * rowHeight
  const extraHeight = milestones === null ? 0 : milestoneHeight
  var containerWidth = Math.floor(document.querySelector(chartContainer).clientWidth)
  if (containerWidth <= 0) {
    var closestCol = document
      .querySelector(chartContainer)
      .closest('[class*="col-"]')

    if (closestCol.clientWidth <= 0) {
      return;
    }
    containerWidth = closestCol.clientWidth
  }
  var width = containerWidth - margin.left - margin.right
  var x = d3
    .scaleTime()
    .domain([timeDomainStart, timeDomainEnd])
    .range([0, width])
    .clamp(true)

  var y = d3
    .scaleBand()
    .domain(lanes)
    .rangeRound([0, height - margin.top - margin.bottom])
    .padding(0.6)

  var xAxis = d3.axisBottom(x)
  var yAxis = d3.axisRight(y).tickPadding(-margin.left).tickSize(0)
  var chartTranslateX = margin.left;

  sortTasks(entries)

  // Build initial chart body
  var svg = d3
    .select(chartContainer)
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
      )

  addXAxis(svg, height, xAxis, extraHeight)
  addXAxisHorizontalLines(svg, height, extraHeight)

  addYAxis(svg, yAxis, taskVersions, extraHeight)
  addYAxisVerticalLines(svg, width)

  addBarsToChart(svg, entries, legend, x, y, highlightVersion, chartContainer, extraHeight);
  addTooltipToBars(svg, legend)


  if (milestones !== null) {
    addMilestones(svg, milestones, height, milestoneHeight)
    addMilestoneMenu(chartContainer, milestones)
  }

  cleanUpChart(svg);
  buildChartKey(chartContainer, legend)
}


// Runtime functions

const chart = document.querySelector("#support-chart")

function buildCharts() {
  if (chart) {
    createChart(
      "#support-chart",
      dataLanes,
      dataLegend,
      dataEntries,
      dataMilestones,
    )
  }
}

function clearCharts() {
  if (chart) {
    chart.innerHTML = ""
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

function chartMenuUpdate() {
  const btn = document.querySelector("#support-chart-menu button")
  const list = btn.parentNode.querySelector("ul")
  if (btn.ariaExpanded === "true") {
    btn.classList.add("support-chart-toggled")
    list.classList.remove("support-chart-hidden")
    list.ariaHidden = "false"
  } else {
    btn.classList.remove("support-chart-toggled")
    list.classList.add("support-chart-hidden")
    list.ariaHidden = "true"
  }

  const currentVersion = list.querySelector('[aria-selected="true"]').dataset.version
  let currentCores = dataLanes
  if (currentVersion !== "all") {
    const milestone = dataMilestones.find((e) => e.id === currentVersion)
    // Set a core subrange. The lane order is reversed, so an off-by-one adjustment is
    // required for the earliest.
    currentCores = dataLanes.slice(
      dataLanes.indexOf(milestone.lanes.highest),
      dataLanes.indexOf(milestone.lanes.lowest) + 1,
    )
  }

  // Recolor elements based on current chart selection
  if (currentVersion) {
    const bars = chart.querySelectorAll(".gantt-chart rect")
    const milestones = chart.querySelectorAll(".gantt-chart path[data-version], .gantt-chart text[data-version]")
    for (let e of bars) {
      // Check `__data__` because we're dealing with a D3-generated bar
      if (currentCores.includes(e.__data__.taskName)) {
        e.removeAttribute("style")
      } else {
        e.setAttribute("style", "fill: var(--color-background-item)")
      }
    }
    for (let e of milestones) {
      if (e.dataset.version === currentVersion || currentVersion == "all") {
        e.classList.remove("support-chart-hidden")
      } else {
        e.classList.add("support-chart-hidden")
      }
    }
  }
}

function chartButtonToggle(set) {
  const btn = document.querySelector("#support-chart-menu button")
  if (set) {
    btn.ariaExpanded = set === "off" ? "false" : "true"
  } else {
    btn.ariaExpanded = btn.ariaExpanded === "true" ? "false" : "true"
  }
  chartMenuUpdate()
}

function chartMenuSelect(item) {
  const list = item.parentNode
  const btn = list.parentNode.querySelector("button")
  list.childNodes.forEach((e) => {
    e.ariaSelected = "false"
    e.classList.remove("support-chart-toggled")
  })
  item.ariaSelected = "true"
  item.classList.add("support-chart-toggled")
  btn.textContent = item.textContent
  chartMenuUpdate()
  chartButtonToggle("off")
}


// Events

window.addEventListener(
  "resize",
  debounce(function () {
    clearCharts()
    buildCharts()
  }, 250),
);

document.addEventListener("click", (e) => {
  const btn = document.querySelector("#support-chart-menu button")
  if (e.target !== btn && btn.ariaExpanded === "true") {
    chartButtonToggle("off")
  }
})

buildCharts();
