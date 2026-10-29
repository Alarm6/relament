import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const html=fs.readFileSync(new URL('../dist/index.html',import.meta.url),'utf8');
const solver=html.split('// CAP_SOLVER_BEGIN')[1].split('// CAP_SOLVER_END')[0];
const create=vm.runInNewContext(solver+';createCapSimulation');
function setup(count=76){
  const bodies=Array.from({length:count},(_,i)=>{const scale=.75+((i*37)%65)/100;return {p:{x:0,y:0,z:0},v:{x:0,y:0,z:0},mass:scale**3,r:.408*scale,baseRadius:.408*scale}});
  const sim=create(bodies);sim.reset(3.15);
  const cfg={bound:3.15,gather:0,feed:false,progress:0,floor:x=>-2.4+(Math.abs(x)/3.15)**3*1.35,mouse:{x:0,y:0,z:0},mouseActive:false,mouseRadius:1.8,mouseSpeed:0};
  return {bodies,sim,cfg};
}
function run(world,seconds,fps=60){for(let i=0;i<seconds*fps;i++)world.sim.advance(1/fps,world.cfg)}
const metrics={};

// A fully populated pile must settle without intersecting or moving at rest.
const pile=setup();run(pile,16);metrics.pile=pile.sim.diagnostics();
assert.ok(metrics.pile.penetration<.003,JSON.stringify(metrics.pile));
assert.ok(metrics.pile.sleeping>=60,JSON.stringify(metrics.pile));
const settled=pile.bodies.map(b=>({...b.p}));run(pile,2);
const restMovement=Math.max(...pile.bodies.map((b,i)=>Math.hypot(b.p.x-settled[i].x,b.p.y-settled[i].y,b.p.z-settled[i].z)));
metrics.restMovement=restMovement;assert.ok(restMovement<.005,`rest movement ${restMovement}`);

// The same physical duration yields the same result at low and high refresh rates.
const low=setup(),high=setup();run(low,8,30);run(high,8,144);
const frameDifference=Math.max(...low.bodies.map((b,i)=>Math.hypot(b.p.x-high.bodies[i].p.x,b.p.y-high.bodies[i].p.y,b.p.z-high.bodies[i].p.z)));
metrics.frameDifference=frameDifference;assert.ok(frameDifference<1e-7,`frame difference ${frameDifference}`);

// Gathering and a strong cursor impulse must retain collision separation.
pile.cfg.gather=1;run(pile,12);metrics.cluster=pile.sim.diagnostics();assert.ok(metrics.cluster.penetration<.003,JSON.stringify(metrics.cluster));
pile.cfg.mouseActive=true;pile.cfg.mouseSpeed=3;run(pile,.5);pile.cfg.mouseActive=false;run(pile,8);metrics.afterFlick=pile.sim.diagnostics();assert.ok(metrics.afterFlick.penetration<.003,JSON.stringify(metrics.afterFlick));

// Contacts remain active in the feed hopper, and all eligible caps can be consumed.
pile.cfg.feed=true;pile.cfg.progress=.2;for(const b of pile.bodies)b.r=b.baseRadius*.53;run(pile,5);metrics.hopper=pile.sim.diagnostics();assert.ok(metrics.hopper.penetration<.003,JSON.stringify(metrics.hopper));
pile.cfg.progress=1.2;run(pile,12);metrics.finished=pile.sim.diagnostics();assert.equal(metrics.finished.visible,0,JSON.stringify(metrics.finished));
console.log(JSON.stringify(metrics,null,2));
