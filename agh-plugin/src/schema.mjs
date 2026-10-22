// Hand-tagged TypeBox schemas. Keeping this dependency-free lets AGH install the plugin as a
// self-contained snapshot; the tags mirror @sinclair/typebox's runtime shape.
const Kind = Symbol.for('TypeBox.Kind')
const Optional = Symbol.for('TypeBox.Optional')

export function stringSchema(options = {}) {
  return { [Kind]: 'String', type: 'string', ...options }
}

export function numberSchema(options = {}) {
  return { [Kind]: 'Number', type: 'number', ...options }
}

export function integerSchema(options = {}) {
  return { [Kind]: 'Integer', type: 'integer', ...options }
}

export function booleanSchema() {
  return { [Kind]: 'Boolean', type: 'boolean' }
}

export function unknownSchema() {
  return { [Kind]: 'Unknown' }
}

export function literalSchema(value) {
  return { [Kind]: 'Literal', const: value, type: typeof value }
}

export function unionSchema(values) {
  return { [Kind]: 'Union', anyOf: values }
}

export function enumerationSchema(...values) {
  return unionSchema(values.map(literalSchema))
}

export function optionalSchema(schema) {
  return { ...schema, [Optional]: 'Optional' }
}

export function arraySchema(items, options = {}) {
  return { [Kind]: 'Array', type: 'array', items, ...options }
}

export function objectSchema(properties, options = {}) {
  return {
    [Kind]: 'Object',
    type: 'object',
    properties,
    required: Object.keys(properties).filter((key) => properties[key]?.[Optional] === undefined),
    additionalProperties: false,
    ...options,
  }
}
