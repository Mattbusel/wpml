//! A small element tree over quick-xml, with the WPML namespace resolved.
//!
//! DJI has shipped the WPML namespace under several URIs: the Cloud API docs
//! use `http://www.dji.com/wpmz/1.0.2`, the MSDK v5 waypoint sample uses
//! `http://www.dji.com/wpmz/1.0.0` and the MSDK v5 flight task sample uses
//! `http://www.corp.com/wpmz/1.0.6`. Matching on the prefix or on one exact
//! URI breaks on one of them, so any namespace whose URI contains `/wpmz/`
//! counts. Elements in it are named `wpml:<local>` here whatever prefix the
//! file used.

use crate::Error;
use quick_xml::events::Event;
use quick_xml::name::ResolveResult;
use quick_xml::NsReader;

/// The KML 2.2 namespace.
pub const KML_NS: &str = "http://www.opengis.net/kml/2.2";

/// One XML element with its text and children. Names in the WPML namespace
/// are `wpml:<local>`, names in the KML namespace (or none) are bare.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Element {
    /// Normalized name, such as `wpml:hoverTime` or `Placemark`.
    pub name: String,
    /// Attributes as written (name, unescaped value).
    pub attrs: Vec<(String, String)>,
    /// Text content, trimmed, with entities resolved.
    pub text: String,
    /// Child elements in document order.
    pub children: Vec<Element>,
}

impl Element {
    /// A leaf element with text.
    pub fn leaf(name: &str, text: impl Into<String>) -> Self {
        Element {
            name: name.into(),
            attrs: Vec::new(),
            text: text.into(),
            children: Vec::new(),
        }
    }

    /// The first child with this normalized name.
    pub fn child(&self, name: &str) -> Option<&Element> {
        self.children.iter().find(|c| c.name == name)
    }

    /// The text of the first child with this name.
    pub fn child_text(&self, name: &str) -> Option<&str> {
        self.child(name).map(|c| c.text.as_str())
    }

    /// Serializes the element (and children) with two space indentation.
    pub fn write_to(&self, out: &mut String, depth: usize) {
        indent(out, depth);
        out.push('<');
        out.push_str(&self.name);
        for (k, v) in &self.attrs {
            out.push(' ');
            out.push_str(k);
            out.push_str("=\"");
            out.push_str(&quick_xml::escape::escape(v.as_str()));
            out.push('"');
        }
        if self.children.is_empty() && self.text.is_empty() {
            out.push_str("/>\n");
            return;
        }
        out.push('>');
        if self.children.is_empty() {
            out.push_str(&quick_xml::escape::escape(self.text.as_str()));
        } else {
            out.push('\n');
            if !self.text.is_empty() {
                indent(out, depth + 1);
                out.push_str(&quick_xml::escape::escape(self.text.as_str()));
                out.push('\n');
            }
            for c in &self.children {
                c.write_to(out, depth + 1);
            }
            indent(out, depth);
        }
        out.push_str("</");
        out.push_str(&self.name);
        out.push_str(">\n");
    }

    /// The element as an XML string.
    pub fn to_xml(&self) -> String {
        let mut s = String::new();
        self.write_to(&mut s, 0);
        s
    }
}

pub(crate) fn indent(out: &mut String, depth: usize) {
    for _ in 0..depth {
        out.push_str("  ");
    }
}

/// True when a namespace URI is a WPML one (`.../wpmz/<version>`).
pub fn is_wpml_ns(uri: &str) -> bool {
    uri.contains("/wpmz/")
}

/// A parsed document: the root element and the WPML namespace URI it used.
#[derive(Debug, Clone)]
pub struct Parsed {
    /// The root element (normally `kml`).
    pub root: Element,
    /// The WPML namespace URI declared in the file, if any.
    pub wpml_ns: Option<String>,
    /// Other `xmlns:*` declarations on the root, kept so unknown elements
    /// in foreign namespaces can be written back.
    pub other_ns: Vec<(String, String)>,
}

/// Parses XML text into an [`Element`] tree.
pub fn parse(text: &str) -> Result<Parsed, Error> {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut reader = NsReader::from_str(text);
    reader.config_mut().trim_text(false);
    let mut stack: Vec<Element> = Vec::new();
    let mut root: Option<Element> = None;
    let mut wpml_ns: Option<String> = None;
    let mut other_ns = Vec::new();
    loop {
        let pos = reader.buffer_position();
        let (res, ev) = reader
            .read_resolved_event()
            .map_err(|e| Error::Xml(format!("{e} after byte {pos}")))?;
        match ev {
            Event::Start(ref s) | Event::Empty(ref s) => {
                let qname = s.name().into_inner().to_string();
                let local = s.local_name().into_inner().to_string();
                let name = normalize(&res, &qname, &local, &mut wpml_ns);
                let mut attrs = Vec::new();
                for a in s.attributes() {
                    let a = a.map_err(|e| Error::Xml(e.to_string()))?;
                    let key = a.key.into_inner().to_string();
                    let value = a
                        .normalized_value(quick_xml::XmlVersion::Implicit1_0)
                        .map_err(|e| Error::Xml(e.to_string()))?
                        .into_owned();
                    if let Some(prefix) = key.strip_prefix("xmlns:") {
                        if is_wpml_ns(&value) {
                            if wpml_ns.is_none() {
                                wpml_ns = Some(value.clone());
                            }
                            continue;
                        }
                        if stack.is_empty() && prefix != "wpml" {
                            other_ns.push((prefix.to_string(), value.clone()));
                            continue;
                        }
                    }
                    if key == "xmlns" && stack.is_empty() {
                        continue;
                    }
                    attrs.push((key, value));
                }
                let el = Element {
                    name,
                    attrs,
                    text: String::new(),
                    children: Vec::new(),
                };
                if matches!(ev, Event::Start(_)) {
                    stack.push(el);
                } else {
                    push_child(&mut stack, &mut root, el)?;
                }
            }
            Event::End(_) => {
                let el = stack
                    .pop()
                    .ok_or_else(|| Error::Xml("unbalanced end tag".into()))?;
                push_child(&mut stack, &mut root, el)?;
            }
            Event::Text(t) => {
                if let Some(top) = stack.last_mut() {
                    top.text.push_str(&t.xml10_content());
                }
            }
            Event::CData(t) => {
                if let Some(top) = stack.last_mut() {
                    top.text.push_str(&t.into_inner());
                }
            }
            Event::GeneralRef(r) => {
                if let Some(top) = stack.last_mut() {
                    if let Ok(Some(c)) = r.resolve_char_ref() {
                        top.text.push(c);
                    } else {
                        let name = r.xml10_content();
                        match quick_xml::escape::resolve_predefined_entity(&name) {
                            Some(s) => top.text.push_str(s),
                            None => {
                                return Err(Error::Xml(format!("unknown entity &{name};")));
                            }
                        }
                    }
                }
            }
            Event::Eof => break,
            _ => {}
        }
    }
    if !stack.is_empty() {
        return Err(Error::Xml("unexpected end of document".into()));
    }
    let mut root = root.ok_or_else(|| Error::Xml("empty document".into()))?;
    trim_all(&mut root);
    Ok(Parsed {
        root,
        wpml_ns,
        other_ns,
    })
}

fn trim_all(e: &mut Element) {
    let t = e.text.trim();
    if t.len() != e.text.len() {
        e.text = t.to_string();
    }
    for c in &mut e.children {
        trim_all(c);
    }
}

fn push_child(stack: &mut [Element], root: &mut Option<Element>, el: Element) -> Result<(), Error> {
    match stack.last_mut() {
        Some(parent) => parent.children.push(el),
        None => {
            if root.is_some() {
                return Err(Error::Xml("more than one root element".into()));
            }
            *root = Some(el);
        }
    }
    Ok(())
}

fn normalize(
    res: &ResolveResult,
    qname: &str,
    local: &str,
    wpml_ns: &mut Option<String>,
) -> String {
    match res {
        ResolveResult::Bound(ns) => {
            let uri = ns.into_inner();
            if is_wpml_ns(uri) {
                if wpml_ns.is_none() {
                    *wpml_ns = Some(uri.to_string());
                }
                format!("wpml:{local}")
            } else if uri == KML_NS || uri.starts_with("http://www.opengis.net/kml/") {
                local.to_string()
            } else {
                qname.to_string()
            }
        }
        ResolveResult::Unbound => local.to_string(),
        // An undeclared `wpml:` prefix still means WPML; anything else is kept as written.
        ResolveResult::Unknown(p) if p == "wpml" => format!("wpml:{local}"),
        ResolveResult::Unknown(_) => qname.to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn any_wpmz_namespace_and_prefix() {
        // The MSDK v5 flight task sample declares http://www.corp.com/wpmz/1.0.6.
        let x = r#"<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:w="http://www.corp.com/wpmz/1.0.6"><Document><w:missionConfig><w:finishAction>goHome</w:finishAction></w:missionConfig></Document></kml>"#;
        let p = parse(x).unwrap();
        assert_eq!(p.wpml_ns.as_deref(), Some("http://www.corp.com/wpmz/1.0.6"));
        let doc = p.root.child("Document").unwrap();
        let mc = doc.child("wpml:missionConfig").unwrap();
        assert_eq!(mc.child_text("wpml:finishAction"), Some("goHome"));
    }

    #[test]
    fn entities_and_round_trip() {
        let x = "<a><wpml:b>x &amp; y &#65;</wpml:b></a>";
        let p = parse(x).unwrap();
        assert_eq!(p.root.child_text("wpml:b"), Some("x & y A"));
        let again = parse(&p.root.to_xml()).unwrap();
        assert_eq!(again.root, p.root);
    }
}
