#!/usr/bin/env node

const bl             = require('bl')
    , split2         = require('split2')
    , list           = require('list-stream')
    , fs             = require('fs')
    , path           = require('path')
    , chalk          = require('chalk')
    , pkgtoId        = require('pkg-to-id')
    , commitStream   = require('commit-stream')
    , gitexec        = require('gitexec')
    , commitToOutput = require('./commit-to-output')
    , groupCommits   = require('./group-commits')
    , collectCommitLabels = require('./collect-commit-labels')
    , isReleaseCommit = require('./groups').isReleaseCommit

    , argv           = require('minimist')(process.argv.slice(2))

    , quiet          = argv.quiet || argv.q
    , simple         = argv.simple || argv.s
    , help           = argv.h || argv.help 

    , pkg            = require('./package.json')
    , debug          = require('debug')(pkg.name)
    , pkgFile        = path.join(process.cwd(), 'package.json')
    , pkgData        = fs.existsSync(pkgFile) ? require(pkgFile) : {}
    , pkgId          = pkgtoId(pkgData)

    , ghId           = {
          user: argv._[0] || pkgId.user || 'nodejs'
        , name: argv._[1] || (pkgId.name && stripScope(pkgId.name)) || 'node'
      }

const gitcmd         = 'git log --pretty=full --since="{{sincecmd}}" --until="{{untilcmd}}"'
    , commitdatecmd  = '$(git show -s --format=%cd `{{refcmd}}`)'
    , untilcmd       = ''
    , refcmd         = argv.a || argv.all ? 'git rev-list --max-parents=0 HEAD' : 'git rev-list --max-count=1 {{ref}}'
    , defaultRef     = '--tags=v*.*.* 2> /dev/null ' +
        '|| git rev-list --max-count=1 --tags=*.*.* 2> /dev/null ' +
        '|| git rev-list --max-count=1 HEAD'

debug(ghId)

if (help) {
  showUsage()
  process.exit(0)
} 

function showUsage () {
  var usage = fs.readFileSync(path.join(__dirname, 'README.md'), 'utf8')
    .replace(/[\s\S]+(## Usage\n[\s\S]*)\n## [\s\S]+/m, '$1')
  if (process.stdout.isTTY) {
    usage = usage
      .replace(/## Usage\n[\s]*/m, '')
      .replace(/\*\*/g, '')
      .replace(/`/g, '')
  }
  process.stdout.write(usage)
} 

function stripScope (name) {
  return name[0] === '@' && name.indexOf('/') > 0 ? name.split('/')[1] : name
}

function replace (s, m) {
  Object.keys(m).forEach(function (k) {
    s = s.replace(new RegExp('\\{\\{' + k + '\\}\\}', 'g'), m[k])
  })
  return s
}


function organiseCommits (list) {
  if (argv['start-ref'] || argv.a || argv.all) {
    if (argv['filter-release'])
      list = list.filter(function (commit) { return !isReleaseCommit(commit.summary) })
    return list
  }

  // filter commits to those _before_ 'working on ...'
  var started = false
  return list.filter(function (commit) {
    if (started)
      return false

    if (isReleaseCommit(commit.summary))
      started = true

    return !started
  })
}


function printCommits (list) {
  var out = list.join('\n') + '\n'

  if (!process.stdout.isTTY)
    out = chalk.stripColor(out)

  process.stdout.write(out)
}


function onCommitList (err, list) {
  if (err)
    throw err

  list = organiseCommits(list)

  collectCommitLabels(list, function (err) {
    if (err)
      throw err

    if (argv.group)
      list = groupCommits(list)

    list = list.map(function (commit) {
      return commitToOutput(commit, simple, ghId)
    })

    if (!quiet)
      printCommits(list)
  })
}


var _startrefcmd = replace(refcmd, { ref: argv['start-ref'] || defaultRef })
  , _endrefcmd   = argv['end-ref'] && replace(refcmd, { ref: argv['end-ref'] })
  , _sincecmd    = replace(commitdatecmd, { refcmd: _startrefcmd })
  , _untilcmd    = argv['end-ref'] ? replace(commitdatecmd, { refcmd: _endrefcmd }) : untilcmd
  , _gitcmd      = replace(gitcmd, { sincecmd: _sincecmd, untilcmd: _untilcmd })

debug('%s', _startrefcmd)
debug('%s', _endrefcmd)
debug('%s', _sincecmd)
debug('%s', _untilcmd)
debug('%s', _gitcmd)

gitexec.exec(process.cwd(), _gitcmd)
  .pipe(split2())
  .pipe(commitStream(ghId.user, ghId.name))
  .pipe(list.obj(onCommitList))
