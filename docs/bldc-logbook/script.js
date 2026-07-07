const STORAGE_KEY = "bldc-logbook-draft-v1";

const editableSelectors = [
  ".meta-card[contenteditable='true']",
  ".editable-block[contenteditable='true']",
  ".timeline-card[contenteditable='true']",
  ".issue-body[contenteditable='true']",
  ".code-note[contenteditable='true']"
];

const statusEl = document.getElementById("draft-status");
const saveButton = document.getElementById("save-draft");
const resetButton = document.getElementById("reset-draft");
const addEntryButton = document.getElementById("add-entry");
const addTimelineButton = document.getElementById("add-timeline");
const addIssueButton = document.getElementById("add-issue");
const timeline = document.getElementById("timeline");
const issueBoard = document.getElementById("issue-board");
const timelineTemplate = document.getElementById("timeline-template");
const issueTemplate = document.getElementById("issue-template");

let saveTimer = null;

function collectDraft() {
  return {
    editables: Array.from(document.querySelectorAll(editableSelectors.join(","))).map((node) => node.innerHTML),
    checks: Array.from(document.querySelectorAll(".checklist input")).map((node) => node.checked),
    timelineHTML: timeline.innerHTML,
    issueHTML: issueBoard.innerHTML
  };
}

function applyDraft(draft) {
  if (!draft) {
    return;
  }

  if (draft.timelineHTML) {
    timeline.innerHTML = draft.timelineHTML;
  }

  if (draft.issueHTML) {
    issueBoard.innerHTML = draft.issueHTML;
  }

  const editables = Array.from(document.querySelectorAll(editableSelectors.join(",")));
  draft.editables?.forEach((html, index) => {
    if (editables[index]) {
      editables[index].innerHTML = html;
    }
  });

  const checks = Array.from(document.querySelectorAll(".checklist input"));
  draft.checks?.forEach((checked, index) => {
    if (checks[index]) {
      checks[index].checked = checked;
    }
  });
}

function setStatus(text) {
  statusEl.textContent = text;
}

function saveDraft() {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(collectDraft()));
  setStatus(`本地草稿已保存 ${new Date().toLocaleTimeString("zh-CN", { hour: "2-digit", minute: "2-digit" })}`);
}

function scheduleSave() {
  setStatus("检测到修改，等待自动保存");
  window.clearTimeout(saveTimer);
  saveTimer = window.setTimeout(saveDraft, 500);
}

function addTimelineCard() {
  const clone = timelineTemplate.content.cloneNode(true);
  timeline.appendChild(clone);
  scheduleSave();
}

function addIssuePair() {
  const clone = issueTemplate.content.cloneNode(true);
  issueBoard.appendChild(clone);
  scheduleSave();
}

function restoreDraft() {
  const raw = localStorage.getItem(STORAGE_KEY);
  if (!raw) {
    setStatus("本地草稿未保存");
    return;
  }

  try {
    applyDraft(JSON.parse(raw));
    setStatus("已恢复本地草稿");
  } catch (error) {
    console.error(error);
    setStatus("草稿恢复失败");
  }
}

function resetDraft() {
  localStorage.removeItem(STORAGE_KEY);
  window.location.reload();
}

document.addEventListener("input", (event) => {
  if (event.target.matches("[contenteditable='true']")) {
    scheduleSave();
  }
});

document.addEventListener("change", (event) => {
  if (event.target.matches(".checklist input")) {
    scheduleSave();
  }
});

saveButton.addEventListener("click", saveDraft);
resetButton.addEventListener("click", resetDraft);
addEntryButton.addEventListener("click", addTimelineCard);
addTimelineButton.addEventListener("click", addTimelineCard);
addIssueButton.addEventListener("click", addIssuePair);

restoreDraft();
