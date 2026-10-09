// SPDX-FileCopyrightText: 2026 James Harton
//
// SPDX-License-Identifier: Apache-2.0


// An absolute touch pad. The centre of the pad is neutral and the command is
// where the finger is relative to it, so the edges are full deflection and a
// thumb can be put straight where it means rather than dragged there.
//
// Neutral is marked by a ring that CSS centres and this never touches. Placing
// it from here worked until the first re-render, when LiveView patched the
// element back to what the template said and the ring vanished.
//
// The cost, and it is a real one: landing a thumb near an edge commands full
// deflection the instant it touches, where a relative pad would have started
// from a stop. `drive_slew` in the balance controller is what ramps that into
// something the robot can take.
export const DrivePad = {
  // The robot reads a drive command at 100 Hz but only needs it to be fresh, and
  // every one of these is a round trip over wifi.
  INTERVAL: 50,

  mounted() {
    this.driving = false
    this.sentAt = 0
    this.pending = null

    this.thumbEl = document.getElementById("pad-thumb")

    this.el.addEventListener("pointerdown", event => {
      this.el.setPointerCapture(event.pointerId)
      this.driving = true
      this.move(event)
    })

    this.el.addEventListener("pointermove", event => {
      if (this.driving) this.move(event)
    })

    for (const name of ["pointerup", "pointercancel", "pointerleave"]) {
      this.el.addEventListener(name, () => this.release())
    }

    // A tab going to the background stops getting pointer events but keeps the
    // socket, so without this a robot could be left driving by a phone in a
    // pocket. The controller's timeout would catch it; this catches it sooner.
    this.onHidden = () => document.hidden && this.release()
    document.addEventListener("visibilitychange", this.onHidden)
  },

  destroyed() {
    document.removeEventListener("visibilitychange", this.onHidden)
  },

  // Half the pad in each direction, so the edges are exactly full deflection and
  // the whole of the screen means something. The axes scale separately: a phone
  // is taller than it is wide, which leaves throttle the less twitchy of the two
  // and steering the more precise.
  centre() {
    const rect = this.el.getBoundingClientRect()

    return {
      x: rect.left + rect.width / 2,
      y: rect.top + rect.height / 2,
      halfWidth: Math.max(1, rect.width / 2),
      halfHeight: Math.max(1, rect.height / 2)
    }
  },

  move(event) {
    const centre = this.centre()
    const x = this.clamp((event.clientX - centre.x) / centre.halfWidth)
    const y = this.clamp((event.clientY - centre.y) / centre.halfHeight)

    this.place(this.thumbEl, event.clientX, event.clientY)

    // Throttled, but the last move always gets sent — dropping it would leave
    // the robot driving at whatever the second-to-last one asked for.
    const now = Date.now()
    if (now - this.sentAt >= this.INTERVAL) {
      this.sentAt = now
      clearTimeout(this.pending)
      this.pushEvent("drive", {x: x, y: y})
    } else {
      clearTimeout(this.pending)
      this.pending = setTimeout(() => {
        this.sentAt = Date.now()
        this.pushEvent("drive", {x: x, y: y})
      }, this.INTERVAL)
    }
  },

  release() {
    if (!this.driving) return

    this.driving = false
    clearTimeout(this.pending)
    this.thumbEl.classList.add("hidden")
    this.pushEvent("release", {})
  },

  place(el, x, y) {
    const rect = this.el.getBoundingClientRect()
    el.style.left = `${x - rect.left}px`
    el.style.top = `${y - rect.top}px`
    el.classList.remove("hidden")
  },

  clamp(value) {
    return Math.max(-1, Math.min(1, value))
  }
}
